/**
 * Copyright (c) 2011, Daniel Strother < http://danstrother.com/ >
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   - Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   - The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
  * Modified by Oleg Artamonov
  * Modified by Mykhailo Lytvyn
 */

#define VERSION "2012-08-11/1"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "usb_serial.h"

#include "oven_ssr.h"
#include "oven_timing.h"
#include "oven_pid.h"
#include "oven_profile.h"
#include "thcouple.h"
#include "lcd.h"

#define gled_on() PORTD |= _BV(5)
#define gled_off() PORTD &= ~_BV(5)
#define buzzer_on() PORTD |= _BV(6)
#define buzzer_off() PORTD &= ~_BV(6)
#define rled_on() PORTD |= _BV(7)
#define rled_off() PORTD &= ~_BV(7)
#define relay_on() PORTD |= _BV(3)
#define relay_off() PORTD &= ~_BV(3)

#define button1() !(PINB & (1<<PB6))
#define button2() !(PINC & (1<<PC6))

uint32_t Boot_Key __attribute__ ((section (".noinit")));
#define MAGIC_BOOT_KEY            0xDC42ACCA
#define BOOTLOADER_START_ADDRESS  0x7000

void Jump_To_Bootloader(void)
{
    // Disable all interrupts
    cli();

    // Set the bootloader key to the magic value and force a reset
    Boot_Key = MAGIC_BOOT_KEY;
    wdt_enable(WDTO_250MS);
    for (;;);
}


volatile uint8_t mode_manual;
volatile int16_t manual_target;
volatile uint8_t manual_target_changed;


#define CMD_RESET   1
#define CMD_GO      2
#define CMD_PAUSE   3
#define CMD_RESUME  4
#define CMD_MANUAL  5

// TODO: currently, only one comm_cmd can be processed per oven_update_4hz
// invocation, so multiple commands received within a ~0.25s window may be lost
volatile uint8_t comm_cmd;


// controller state

#define ST_FAULT    0
#define ST_IDLE     1
#define ST_RUN      2
#define ST_DONE     3
#define ST_PAUSE    4
#define ST_MANUAL   5

const char *state_names[6] = { "Fault", "Idle", "Run", "Done", "Pause", "Manual" };

uint8_t state = ST_FAULT;

int16_t target;
uint16_t time;
uint8_t buzzer;
uint8_t fw_update;

volatile uint8_t lcd_update_now;
volatile uint8_t button_pressed;
volatile uint8_t profile_num;

void fault(void)
{
    state = ST_FAULT;
    ssr_fault();
}

void oven_output(uint8_t heater)
{
    ssr_set(heater);
}

void oven_input(int16_t *heater)
{
    *heater = thcouple_read();
}

void oven_setup(void)
{
    state = ST_IDLE;

    mode_manual	= 0;
    manual_target_changed = 0;
    comm_cmd = 0;
    target = 0;
    time = 0;
    buzzer = 0;
    fw_update  = 0;


    lcd_update_now = 0;
    button_pressed = 0;
    profile_num = 0;

    pid_reset();
    profile_reset(0);
    ssr_setup();
    adc_setup();
    timing_setup(); // timing setup last, since it enables timer interrupts (which invoke the update functions below)
}

void oven_update_120hz(void)
{
    ssr_update();

    // process buttons

    if (!button_pressed)
    {
        if (button1()) // button 1 (red) pressed
        {
            button_pressed = 1;
            switch (state)
            {
                case ST_FAULT:
                    mode_manual = 0;
                    comm_cmd = CMD_RESET;
                    break;
                case ST_IDLE:
                    if (!mode_manual)
                        comm_cmd = CMD_GO;
                    else
                        comm_cmd = CMD_MANUAL;

                    if (fw_update)
                    {
                        lcd_put_two_rows("FIRMWARE UPDATE", "Atmel FLIP v/USB");
                        usb_serial_flush_output();
                        Jump_To_Bootloader();
                    }
                    break;
                case ST_RUN:
                    comm_cmd = CMD_PAUSE;
                    break;
                case ST_DONE:
                    comm_cmd = CMD_RESET;
                    break;
                case ST_PAUSE:
                    comm_cmd = CMD_RESET;
                    break;
                case ST_MANUAL:
                    if (!manual_target_changed)
                    {
                        comm_cmd = CMD_PAUSE;
                    }
                    else
                    {
                        target = manual_target;
                        manual_target_changed = 0;
                    }
                    break;
                default:
                    break;
            }
            lcd_update_now = 1;
        }

        if (button2()) // button 2 (green) pressed
        {
            button_pressed = 1;
            switch (state)
            {
                case ST_FAULT:
                    break;
                case ST_IDLE:
                    profile_num++;
                    mode_manual = 0;
                    fw_update = 0;

                    if (profile_num == (PROFILES+2))
                        profile_num = 0;

                    if (profile_num < PROFILES)
                        profile_reset(profile_num);

                    if (profile_num == PROFILES)
                    {
                        mode_manual = 1;
                        manual_target = 200; // 50 *C
                    }

                    if (profile_num == PROFILES+1)
                    {
                        fw_update = 1;
                    }
                    break;
                case ST_RUN:
                    break;
                case ST_DONE:
                    break;
                case ST_PAUSE:
                    comm_cmd = CMD_RESUME;
                    break;
                case ST_MANUAL:
                    manual_target += 20;
                    if (manual_target > 1000) // 250 *C
                        manual_target = 200; // 50 *C
                    manual_target_changed = 1;
                    break;
                default:
                    break;
            }
            lcd_update_now = 1;
        }
    }
    else
    {
        if ( button2() && (state == ST_MANUAL) ) // button 2 (green) still pressed, manual mode
        {
            button_pressed++;
            if (button_pressed > 59)
                button_pressed = 0;
        }

        if ( !(button1() | button2()) ) // no button pressed
        {
            button_pressed++;
            if (button_pressed > 10)
                button_pressed = 0;
        }
    }
}

char tx_msg[255];
volatile uint8_t tx_len = 0;
volatile uint8_t lcd_update_1hz = 0;
volatile uint8_t led_blink = 0;

void oven_update_4hz(void)
{
    int16_t temp;
    uint8_t step;
    uint16_t time;

    oven_input(&temp);

    if(comm_cmd != 0)
    {
        switch(comm_cmd)
        {
            case CMD_RESET:
                profile_reset(0);
                pid_reset();
                state = ST_IDLE;
                mode_manual = 0;
                target = 0;
                profile_num = 0;
                break;
            case CMD_GO:
                if(state == ST_IDLE) {
                    state = ST_RUN;
                }
                break;
            case CMD_PAUSE:
                if((state == ST_RUN) || (state == ST_MANUAL)) {
                    state = ST_PAUSE;
                }
                break;
            case CMD_RESUME:
                if(state == ST_PAUSE) {
                    if (!mode_manual)
                        state = ST_RUN;
                    else
                        state = ST_MANUAL;
                }
                break;
            case CMD_MANUAL:
                if (state == ST_IDLE) {
                    target		= manual_target;
                    state		= ST_MANUAL;
                }
                break;
            default:
                fault();
        }

        comm_cmd = 0;
    }

    switch(state)
    {
        case ST_IDLE:
            rled_off();
            relay_off();
            break;
        case ST_RUN:
            rled_on();
            if(profile_update(&target, &step, &time, &buzzer))
            {
                state = ST_DONE;
                buzzer = 8;
            }
            if ((target - temp) > 100) // 25 *C difference
                fault();
            break;
        case ST_PAUSE:
            rled_on();
            // hold target
            break;
        case ST_DONE:
            rled_off();
            relay_off();
            target = 0;
            break;
        case ST_FAULT:
            if (led_blink)
            {
                rled_off();
                led_blink = 0;
            }
            else
            {
                rled_on();
                led_blink = 1;
            }
            break;
        case ST_MANUAL:
            rled_on();
            break;
        default:
            fault();
    }

    oven_output(pid_update(temp, target));

    lcd_update_1hz++;
    char lcd_str[16];
    char lcd_degree = 0xDF;

    if ((lcd_update_1hz == 4) || (lcd_update_now))
    {
        if (buzzer != 0)
        {
            if (buzzer % 2 == 0)
                buzzer_on();
            else
                buzzer_off();

            buzzer--;

            relay_on();
        }

        lcd_clrscr();

        sprintf_P(lcd_str, PSTR("%s %d%cC"), state_names[state], temp>>2, lcd_degree);

        lcd_puts(lcd_str);

        lcd_gotoxy(0, 1);

        if (state == ST_RUN)
        {
            //	2/7 150 s 540 *C -- step, time until next step, target temperature
            sprintf_P(lcd_str, PSTR("%d/%d %ds %d%cC"), step, STEPS, time, target>>2, lcd_degree);
            lcd_puts(lcd_str);
        }

        if (state == ST_MANUAL)
        {
            if (!manual_target_changed)
                sprintf_P(lcd_str, PSTR("Target: %d%cC"), target>>2, lcd_degree);
            else
                sprintf_P(lcd_str, PSTR("%d%cC->%d%cC?"), target>>2, lcd_degree, manual_target>>2, lcd_degree);

            lcd_puts(lcd_str);
        }

        if (state == ST_IDLE)
        {
            switch (profile_num)
            {
                case 0:		//0123456789ABCDEF
                    lcd_puts("1.Lead (fast)");
                    break;
                case 1:
                    lcd_puts("2.Lead (slow)");
                    break;
                case 2:
                    lcd_puts("3.Lead-free");
                    break;
                case 3:
                    lcd_puts("4.Manual control");
                    break;
                case 4:
                    lcd_puts("5.USB F/W update");
                    break;
                default:
                    lcd_puts("UNKNOWN PROFILE");
                    break;
            }
        }

        if (state == ST_FAULT)
        {
            sprintf_P(lcd_str, PSTR("TG%d%cC T%d%cC"), target>>2, lcd_degree, temp>>2, lcd_degree);
            
            lcd_put_two_rows("CHECK TH.COUPLE!", lcd_str);                        
        }

        if (state == ST_PAUSE)
        {
            lcd_puts("1 stop, 2 resume");
        }

        lcd_update_1hz = 0;
        lcd_update_now = 0;
    }

    time++;
}

char rx_msg[255];
uint8_t rx_cnt;

void process_message(const char *msg)
{
    // this is a ridiculously expensive function to invoke - a more efficient
    // command parser could be implemented, or a binary protocol established -
    // but, we're not expecting a lot of command traffic in this application,
    // and all of the timing-critical routines are handled by interrupts, so
    // there isn't a lot of downside to this expensive-but-easy implementation

    uint16_t eeprom_data;
    char response[50];
    uint8_t resplen;

    if(strcmp_P(msg,PSTR("reset")) == 0) {
        comm_cmd = CMD_RESET;
    } else if(strcmp_P(msg,PSTR("go")) == 0) {
        comm_cmd = CMD_GO;
    } else if(strcmp_P(msg,PSTR("pause")) == 0) {
        comm_cmd = CMD_PAUSE;
    } else if(strcmp_P(msg,PSTR("resume")) == 0) {
        comm_cmd = CMD_RESUME;
    } else if(strcmp_P(msg,PSTR("update")) == 0) { // update firmware via USB        
        lcd_put_two_rows("FIRMWARE UPDATE", "RESTART TO ABORT");
        usb_serial_write((void*)"* Update firmware with Atmel FLIP", 33);
        usb_serial_flush_output();
        Jump_To_Bootloader();
    } else if(sscanf_P(msg,PSTR("coeff: %d"), &eeprom_data)) { // TH couple correction coefficient, default 31
        set_coeff(eeprom_data);
        resplen = sprintf_P(response, PSTR("* Coeff: %d\n"), th_coeff);
        usb_serial_write((void*)response, resplen);
    } else if(sscanf_P(msg,PSTR("shift: %d"), &eeprom_data)) { // TMP37 cold junction zero shift, default 0
        set_zero(eeprom_data);
        resplen = sprintf_P(response, PSTR("* Shift: %d\n"), zero_shift);
        usb_serial_write((void*)response, resplen);
    } else if(strcmp_P(msg,PSTR("coeff?")) == 0) {
        resplen = sprintf_P(response, PSTR("* Coeff: %d\n"), th_coeff);
        usb_serial_write((void*)response, resplen);
    } else if(strcmp_P(msg,PSTR("shift?")) == 0) {
        resplen = sprintf_P(response, PSTR("* Shift: %d\n"), zero_shift);
        usb_serial_write((void*)response, resplen);
    }
}

void avr_init(void)
{
    // If the reset source was the bootloader and the key is correct, clear it and jump to the bootloader
    if ((MCUSR & (1<<WDRF)) && (Boot_Key == MAGIC_BOOT_KEY))
    {
        Boot_Key = 0;
        ((void (*)(void))BOOTLOADER_START_ADDRESS)();
    }

    SPCR &= ~(1<<SPE);

    // Pullups
    PINB |= (1<<PB6);
    PINC |= (1<<PC6)|(1<<PC7);
    
    // LED status lights
    DDRD |= (1<<PD5)|(1<<PD6)|(1<<PD7);
    DDRD |= (1<<PD3);
    
    rled_on();
    relay_off();
    
    usb_init();
}

void show_greetings_msg(void)
{
    lcd_clrscr();

    lcd_put_two_rows_animate("REFLOW  OVEN", " CONTROLLER ");
    _delay_ms(500);
    
    lcd_put_two_rows_animate("Mykhailo Lytvyn", "   lytvyn.at   ");
    _delay_ms(500);

    lcd_put_two_rows("FIRMWARE VERSION", VERSION);

}

// program entry point
int main(void)
{
    avr_init();
    
    lcd_init(LCD_DISP_ON);
    
    show_greetings_msg();
    
    oven_setup();

    // wait an arbitrary bit for the host to complete its side of the init
    _delay_ms(1000);

    // clear any stale packets
    usb_serial_flush_input();

    rled_off();
    gled_on();
    
    rx_cnt = 0;

    int8_t ret;
    while(1)
    {
        // receive individual characters from the host
        while( (ret = usb_serial_getchar()) != -1)
        {
            // all commands are terminated with a new-line
            if(ret == '\n') 
            {
                // only process commands that haven't overflowed the buffer
                if(rx_cnt > 0 && rx_cnt < 255) 
                {
                    rx_msg[rx_cnt] = '\0';
                    process_message(rx_msg);
                }
                rx_cnt = 0;
            } else {
                // buffer received characters
                rx_msg[rx_cnt] = ret;
                if(rx_cnt != 255) rx_cnt++;
            }
        }
    }
}



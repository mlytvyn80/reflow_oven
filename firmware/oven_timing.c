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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "oven_timing.h"

// implemented in main program (ovencon.c)
extern void oven_update_120hz(void);
extern void oven_update_4hz(void);

uint8_t divider;

void timing_setup(void)
{
    divider = 0;

    // CTC with ICRn TOP, clk/8
    //    ICR1    = 16807; // 16MHz/(8*16807) = 119Hz (slightly under mains frequency; will sync with external interrupt)
    //ICR1    = 16667; // 119.998 Hz (no external sync)
    ICR1 = 20202; // 99 Hz with external sync
    TCCR1A  = 0;
    TCCR1B  = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
    TCCR1C  = 0;
    OCR1A   = 18000; // update outputs a little while before next zero-cross
    TIMSK1  = _BV(OCIE1A); // enable OCRA1 interrupt

    // enable external interrupt (INT0/PD0 and INT1/PD1; falling edge)
    // TODO: not currently used; requires extra hardware to sample mains
    // (e.g. a low-voltage transformer)

    DDRD    &= ~(_BV(0));
    EICRA = _BV(ISC01);
    EIMSK = _BV(INT0);

    //    PORTD   |= _BV(0) | _BV(1); // pull-up
    //    EICRA   = _BV(ISC01) | _BV(ISC11);
    //    EIMSK   = _BV(INT0) | _BV(INT1);
}

// timer interrupt
ISR(TIMER1_COMPA_vect)
{
    oven_update_120hz();

    // re-enable interrupts
    sei();

    // execute control update every 30 steps (120/30 = 4 Hz)
    divider++;

    if(divider == 30)
    {
        divider = 0;
        oven_update_4hz();
    }
}

// AC zero-cross interrupts - just clear timer
ISR(INT0_vect) { TCNT1 = 0; }
// ISR(INT1_vect) { TCNT1 = 0; }



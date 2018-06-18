#include "thcouple.h"

uint16_t EEMEM th_coeff_ee;
uint16_t EEMEM zero_shift_ee;

// 12-bit temperature (0.25 deg/div)
int16_t thcouple_read(void)
{
    ADCSRB &= ~(1<<MUX5); // ADC0 channel - thermocouple
    ADCSRA |= (1<<ADSC); // start conversion
    do {}
    while ( !(ADCSRA & (1<<ADIF)) );
    ADCSRA |= (1<<ADIF); // clear ADIF flag

    // 41 uV/*C, K = 196 -> 41*196 = 8.036 mV/*C / 2.5 mV/div -> 3.2144 div/*C -> 0.311 *C/div
    uint16_t th_temp = ADC;

    th_temp *= th_coeff;
    th_temp /= 100; // 1 *C/div
    th_temp <<= 2; // 1/4 *C/div


    ADCSRB |= (1<<MUX5); // ADC8 channel - TMP37 (cold junction)
    ADCSRA |= (1<<ADSC); // start conversion
    do {}
    while ( !(ADCSRA & (1<<ADIF)) );
    ADCSRA |= (1<<ADIF); // clear ADIF flag

    // 20 mV/*C, 0V at 0*C -> 1/8 *C/div
    uint16_t tmp37_temp = ADC;
    tmp37_temp >>= 1; // 1/4 *C/div
    tmp37_temp -= zero_shift;

    th_temp += tmp37_temp;

    return th_temp;
}

void adc_setup(void)
{
    // ADC @ 250 kHz single conversion mode, 2.56V ARef
    DIDR0 = (1<<ADC0D); // thermocouple
    DIDR2 = (1<<ADC8D); // TMP37 cold junction sensor
    ADMUX = (1<<REFS0) | (1<<REFS1);
    ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2);
    ADCSRB = (1<<ADHSM);

    ADCSRA |= (1<<ADSC); // dummy conversion
    do {}
    while ( !(ADCSRA & (1<<ADIF)) );
    ADCSRA |= (1<<ADIF); // clear ADIF flag

    th_coeff = eeprom_read_word(&th_coeff_ee);
    zero_shift = eeprom_read_word(&zero_shift_ee);
}


void set_coeff(uint16_t coeff)
{
    eeprom_update_word(&th_coeff_ee, coeff);
    th_coeff = coeff;
}

void set_zero(uint16_t zero)
{
    eeprom_update_word(&zero_shift_ee, zero);
    zero_shift = zero;
}

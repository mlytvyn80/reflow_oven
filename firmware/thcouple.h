#ifndef THCOUPLE_H
#define THCOUPLE_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/eeprom.h>

// 12-bit temperature (0.25 deg/div)
int16_t thcouple_read(void);
void adc_setup(void);

void set_coeff(uint16_t coeff);
void set_zero(uint16_t zero);

uint16_t th_coeff;
uint16_t zero_shift;

#endif
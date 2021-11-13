#pragma once

// internal adc channels:
// 0 - vbat
// 1 - vref
#define ADC_CHANNELS 2

typedef struct battery_adc_def battery_adc_def_t;

void adc_init();
float adc_read(int channel);
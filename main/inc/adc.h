#ifndef ADC_H
#define ADC_H

#include "gd32f3x0.h"

void adc_config(unsigned int channel_group);
uint16_t adc_channel_sample(uint8_t channel);

#endif
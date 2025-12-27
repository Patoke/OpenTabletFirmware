/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "adc.h"
#include "drv_usb_hw.h"
#include "gd32f3x0_adc.h"

/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(unsigned int channel_group)
{
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);

    adc_regular_channel_config(0, channel_group, 5);

    /* ADC external trigger config */
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);

    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);

    /* enable ADC interface */
    adc_enable();

    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
}

/*!
    \brief      ADC channel sample
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t adc_channel_sample(uint8_t channel)
{
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1); // @note: custom
    
    /* ADC regular channel config */
    adc_regular_channel_config(0U, channel, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read());
}

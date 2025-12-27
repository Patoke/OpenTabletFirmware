/*!
    \file    gd32f3x0_hw.c
    \brief   USB hardware configuration for GD32F3x0

    \version 2025-01-01, V2.5.0, firmware for GD32F3x0
*/

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

#include "drv_usb_hw.h"
#include "pen.h"

volatile static float count_1us = 0;
volatile static float count_1ms = 0;

__IO uint32_t delay_time = 0U;
__IO uint32_t timer_prescaler = 0U;
__IO uint32_t usbfs_prescaler = 0U;

/* local function prototypes ('static') */
static void hw_time_set(uint8_t unit);
static void hw_delay(uint32_t ntime, uint8_t unit);

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* systick clock source is from HCLK/8 */
    systick_clksource_set(SYSTICK_CLKSOURCE_HCLK_DIV8);
    count_1us = (float)SystemCoreClock / 8000000;
    count_1ms = (float)count_1us * 1000;
}

/*!
    \brief      configure USB clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_rcu_config(void)
{
    uint32_t system_clock = rcu_clock_freq_get(CK_SYS);

    if(48000000U == system_clock) {
        usbfs_prescaler = RCU_USBFS_CKPLL_DIV1;
        timer_prescaler = 3U;
    } else if(72000000U == system_clock) {
        usbfs_prescaler = RCU_USBFS_CKPLL_DIV1_5;
        timer_prescaler = 5U;
    } else if(96000000U == system_clock) {
        usbfs_prescaler = RCU_USBFS_CKPLL_DIV2;
        timer_prescaler = 7U;
    } else {
        /* reserved */
    }

    rcu_usbfs_clock_config(usbfs_prescaler);

    rcu_periph_clock_enable(RCU_USBFS);
}

/*!
    \brief      configure USB interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_intr_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2); // @note: extra

    nvic_irq_enable(USBFS_IRQn, 3U, 0U);

    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_18);
    exti_init(EXTI_18, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_18);

    nvic_irq_enable(USBFS_WKUP_IRQn, 0U, 0U);
}

/*!
    \brief      initializes delay unit using Timer2
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_init(void)
{
    /* configure the priority group to 2 bits */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

    /* enable the TIM2 global interrupt */
    nvic_irq_enable(TIMER2_IRQn, 1U, 0U);

    rcu_periph_clock_enable(RCU_TIMER2);
}

void usb_vbus_config(void)
{
    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_7);

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_15);
    gpio_bit_reset(GPIOC, GPIO_PIN_15);

    // @note: unknown function, manually inlined
    gpio_bit_reset(GPIOC, GPIO_PIN_15);

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_bit_reset(GPIOA, GPIO_PIN_4);

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_5 | GPIO_PIN_6);
    gpio_bit_set(GPIOA, GPIO_PIN_5);
    gpio_bit_reset(GPIOA, GPIO_PIN_6);

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    gpio_bit_reset(GPIOB, GPIO_PIN_0);
    gpio_bit_reset(GPIOB, GPIO_PIN_1);
    gpio_bit_reset(GPIOB, GPIO_PIN_2);

    // @note: unknown function, manually inlined, what is 0xFF00?
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 0xFF00);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 0xFF00);
    gpio_port_write(GPIOB, 0xFF00);

    // function code
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_8);

    gpio_bit_set(GPIOA, GPIO_PIN_8);

    /* delay is need for stabilizing the vbus low in reset condition,
     * when vbus = 1 and reset-button is pressed by user
     */
    usb_mdelay(200);

    gpio_bit_reset(GPIOA, GPIO_PIN_8);

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_15);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_15);

    init_timer1();
    // @note: unknown function, manually inlined
    timer_channel_output_pulse_value_config(TIMER1, 0, 899U);

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_14);
    gpio_bit_set(GPIOC, GPIO_PIN_14);

    // function code
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    gpio_bit_set(GPIOA, GPIO_PIN_6);
    gpio_bit_set(GPIOA, GPIO_PIN_7);

    init_timer2();

    // @note: unknown function, manually inlined
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER2_IRQn, 0, 1u);

    init_timer5();
}

/*!
    \brief      delay in microseconds
    \param[in]  usec: value of delay required in microseconds
    \param[out] none
    \retval     none
*/
void usb_udelay(const uint32_t usec)
{
    uint32_t ctl;

    /* reload the count value */
    SysTick->LOAD = (uint32_t)(usec * count_1us);
    /* clear the current count value */
    SysTick->VAL = 0x0000U;
    /* enable the systick timer */
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    /* wait for the COUNTFLAG flag set */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    {
        ;
    }
}

/*!
    \brief      delay in milliseconds
    \param[in]  msec: value of delay required in milliseconds
    \param[out] none
    \retval     none
*/
void usb_mdelay(const uint32_t msec)
{
    uint32_t ctl;

    /* reload the count value */
    SysTick->LOAD = (uint32_t)(msec * count_1ms);
    /* clear the current count value */
    SysTick->VAL = 0x0000U;
    /* enable the systick timer */
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    /* wait for the COUNTFLAG flag set */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    {
        ;
    }
}
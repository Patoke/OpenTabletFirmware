#include "drv_usb_core.h"
#include "drv_usb_dev.h"
#include "drv_usb_hw.h"
#include "gd32f3x0_rcu.h"
#include "gd32f3x0_timer.h"
#include "standard_hid_core.h"
#include "usb_ch9_std.h"
#include "usbd_core.h"

#include "pen.h"
#include "usbd_transc.h"
#include <string.h>

usb_core_driver hid_tablet;

const usb_hid_desc_tablet_config_set tablet_hid_config_desc = {
    .config =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_config),
            .bDescriptorType = USB_DESCTYPE_HID
        },
        .wTotalLength         = sizeof(usb_hid_desc_tablet_config_set),
        .bNumInterfaces       = 0x00U,
        .bConfigurationValue  = 0x01U,
        .iConfiguration       = 0x22U,
        .bmAttributes         = 0x24U,
        .bMaxPower            = 0x00U
    },

    .hid_ep_report_in =
    {
        .header =
        {
            .bLength = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress = 0x81U,
        .bmAttributes = 0x03U,
        .wMaxPacketSize = 0x40U,
        .bInterval = 0x02U
    },
    
    .hid_ep_report_out =
    {
        .header =
        {
            .bLength = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress = 0x01U,
        .bmAttributes = 0x03U,
        .wMaxPacketSize = 0x08U,
        .bInterval = 0x01U
    },

    .hid_pen_interface =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber     = 0x01U,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x01U,
        .bInterfaceClass      = USB_HID_CLASS,
        .bInterfaceSubClass   = 0x01U,
        .bInterfaceProtocol   = 0x02U,
        .iInterface           = 0x00U
    },

    .hid_penhid =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_hid),
            .bDescriptorType = USB_DESCTYPE_HID
        },
        .bcdHID               = 0x0110U,
        .bCountryCode         = 0x00U,
        .bNumDescriptors      = 0x01U,
        .bDescriptorType      = USB_DESCTYPE_REPORT,
        .wDescriptorLength    = 0x79U
    },

    .hid_ep_pen_in =
    {
        .header =
        {
            .bLength = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress = 0x82U,
        .bmAttributes = 0x03U,
        .wMaxPacketSize = 0x10U,
        .bInterval = 0x02U
    },

    .hid_buttons_interface =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber     = 0x02U,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x01U,
        .bInterfaceClass      = USB_HID_CLASS,
        .bInterfaceSubClass   = 0x00U,
        .bInterfaceProtocol   = 0x00U,
        .iInterface           = 0x00U
    },

    .hid_buttonshid =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_hid),
            .bDescriptorType = USB_DESCTYPE_HID
        },
        .bcdHID               = 0x0110U,
        .bCountryCode         = 0x00U,
        .bNumDescriptors      = 0x01U,
        .bDescriptorType      = USB_DESCTYPE_REPORT,
        .wDescriptorLength    = 0x019FU
    },

    .hid_ep_keys_in =
    {
        .header =
        {
            .bLength = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress = 0x83U,
        .bmAttributes = 0x03U,
        .wMaxPacketSize = 0x10U,
        .bInterval = 0x02U
    }
};

void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    
    rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_TIMER16);

    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
}

void com_usart_init(void)
{
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);
    /* connect port to USART TX */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

    /* connect port to USART RX */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 921600U);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_enable(USART0);
}

void usb_init(usb_core_driver* udev)
{
    usb_basic_init(&udev->bp, &udev->regs);

    udev->dev.cur_status = (uint8_t)USBD_DEFAULT;

    usb_globalint_disable(&udev->regs);

    usb_core_init(udev->bp, &udev->regs);
    usb_curmode_set(&udev->regs, DEVICE_MODE);

    usb_dev_disconnect(udev);
    usb_mdelay(3);

    usb_devcore_init(udev);

    usbd_connect(udev);
    usb_mdelay(3);

    usb_globalint_enable(&udev->regs);
}

void init_timer1(void)
{
    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocintpara;

    timer_deinit(TIMER1);

    timer_initpara.prescaler = 719;
    timer_initpara.alignedmode = 0;
    timer_initpara.counterdirection = 0;
    timer_initpara.clockdivision = 0;
    timer_initpara.period = 999;
    timer_initpara.repetitioncounter = 0;

    timer_init(TIMER1, &timer_initpara);

    /* CH0 configuration in PWM mode1 */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 799U);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    timer_enable(TIMER1);
}

void init_timer2(void)
{
    timer_parameter_struct timer_initpara;

    timer_deinit(TIMER2);

    timer_initpara.prescaler = 35999;
    timer_initpara.alignedmode = 0;
    timer_initpara.counterdirection = 0;
    timer_initpara.clockdivision = 0;
    timer_initpara.period = 199;
    timer_initpara.repetitioncounter = 0;

    timer_init(TIMER1, &timer_initpara);

    timer_auto_reload_shadow_enable(TIMER2);
    timer_interrupt_flag_clear(TIMER2, TIMER_INT_UP);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);

    timer_disable(TIMER2);
}

void init_timer5(void)
{
    timer_parameter_struct timer_initpara;

    timer_deinit(TIMER2);

    timer_initpara.prescaler = 40000;
    timer_initpara.alignedmode = 0;
    timer_initpara.counterdirection = 0;
    timer_initpara.clockdivision = 0;
    timer_initpara.period = 65535;
    timer_initpara.repetitioncounter = 0;

    timer_init(TIMER5, &timer_initpara);

    timer_auto_reload_shadow_enable(TIMER5);
    timer_interrupt_flag_clear(TIMER5, TIMER_INT_UP);
    timer_interrupt_enable(TIMER5, TIMER_INT_UP);

    timer_enable(TIMER5);
}

/*!
    \brief      initialize the HID device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t tablet_init(usb_dev *udev, uint8_t config_index)
{
    static tablet_hid_handler hid_handler;

    memset((void *)&hid_handler, 0U, sizeof(tablet_hid_handler));

    udev->dev.class_data[USBD_HID_INTERFACE] = (void *)&hid_handler;

    usbd_ep_setup(udev, &(tablet_hid_config_desc.hid_ep_report_out));
    
    /* prepare receive data */
    usbd_ep_recev(udev, 1, hid_handler.data, 8U);

    usbd_ep_setup(udev, &(tablet_hid_config_desc.hid_ep_report_in));
    usbd_ep_setup(udev, &(tablet_hid_config_desc.hid_ep_pen_in));
    usbd_ep_setup(udev, &(tablet_hid_config_desc.hid_ep_keys_in));

    return USBD_OK;
}

/*!
    \brief      deinitialize the HID device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t tablet_deinit(usb_dev *udev, uint8_t config_index)
{
    /* deinitialize HID endpoints */
    usbd_ep_clear(udev, EP1_IN);
    usbd_ep_clear(udev, EP1_OUT);
    usbd_ep_clear(udev, EP2_IN);
    usbd_ep_clear(udev, EP3_IN);

    return USBD_OK;
}

// @Patoke notes:
//  adc channel 7 is pen data
//  adc channel 0, 1 and 3 are button data (on tablet)

int main(void)
{
    // @note: in the Gaomon firmware, this doesn't use the __DSB instruction
    nvic_vector_table_set(0x8000000, 0x4000);

    rcu_config();
    com_usart_init();
    usb_rcu_config();

    usb_init(&hid_tablet);
    
    usb_intr_config();

    usb_vbus_config();

    // @note: fmc related dumb stuff, finish implementing

    // @note: random ahh bool written here, ignore
    
    // @note: function inlined manually    
    // wait until device is fully configurated
    for (int i = 0; hid_tablet.dev.cur_status != USBD_CONFIGURED; i++)
    {
        usb_udelay(3000);
        if (i >= 1000)
        {
            break;
        }   
    }

    // input loop
    while (1)
    {
        do_sample();

        // @note: lots of code used to send hid reports for express buttons and the ring touch sensor, please implement

        // grab_info(&pen_data, &n3_2, &out_hid_packet)
    }

    return 1;
}
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2018 Seb Holzapfel <schnommus@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \addtogroup Examples
 *
 * This example implements a USB Human Interface Device (HID)
 * to demonstrate the use of the USB device stack.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usb-hid-keys.h"
/* Systick interrupt frequency, Hz */
#define SYSTICK_FREQUENCY 100

/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY 14000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0101    /* Program version */

bool g_usbd_is_connected = false;
usbd_device *g_usbd_dev = 0;

static const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {

				0x05, 0x01,	/* Usage Page (Generic Desktop)      */
				0x09, 0x06,	/* Usage (Keyboard)                  */
				0xA1, 0x01,	/* Collection (Application)          */
				0x05, 0x07,	/* Usage Page (Keyboard)             */
				0x19, 224,	/* Usage Minimum (224)               */
				0x29, 231,	/* Usage Maximum (231)               */
				0x15, 0x00,	/* Logical Minimum (0)               */
				0x25, 0x01,	/* Logical Maximum (1)               */
				0x75, 0x01,	/* Report Size (1)                   */
				0x95, 0x08,	/* Report Count (8)                  */
				0x81, 0x02,	/* Input (Data, Variable, Absolute)  */
				0x81, 0x01,	/* Input (Constant)                  */
				0x19, 0x00,	/* Usage Minimum (0)                 */
				0x29, 101,	/* Usage Maximum (101)               */
				0x15, 0x00,	/* Logical Minimum (0)               */
				0x25, 101,	/* Logical Maximum (101)             */
				0x75, 0x08,	/* Report Size (8)                   */
				0x95, 0x06,	/* Report Count (6)                  */
				0x81, 0x00,	/* Input (Data, Array)               */
				0x05, 0x08,	/* Usage Page (LED)                  */
				0x19, 0x01,	/* Usage Minimum (1)                 */
				0x29, 0x05,	/* Usage Maximum (5)                 */
				0x15, 0x00,	/* Logical Minimum (0)               */
				0x25, 0x01,	/* Logical Maximum (1)               */
				0x75, 0x01,	/* Report Size (1)                   */
				0x95, 0x05,	/* Report Count (5)                  */
				0x91, 0x02,	/* Output (Data, Variable, Absolute) */
				0x95, 0x03,	/* Report Count (3)                  */
				0x91, 0x01,	/* Output (Constant)                 */
				0xC0	/* End Collection                    */
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 8,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 2, /* Keyboard */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Tomu",
	"HID Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

    /* Dirty way to know if we're connected */
    g_usbd_is_connected = true;

	return 1;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
}

void usb_isr(void)
{
    usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
    while(1);
}

void sys_tick_handler(void)
{
    static int x = 0;
	static int dir = 1;
//	[modifier, reserved, Key1, Key2, Key3, Key4, Key6, Key7]
	static uint8_t report[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    if(g_usbd_is_connected) {
		x+=dir;
        if (x > 30){
                report[0] = KEY_MOD_LCTRL | KEY_MOD_LALT; // keyboard
                report[2] = KEY_T; // 'T'
                usbd_ep_write_packet(g_usbd_dev, 0x81, report, sizeof(report));
            gpio_toggle(LED_RED_PORT, LED_RED_PIN);
			dir=-dir;
        }
		else if (x < -30){
                report[0] = 0; // keyboard
                report[2] = 0;
                usbd_ep_write_packet(g_usbd_dev, 0x81, report, sizeof(report));
            gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
			dir=-dir;
        }
        //usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 4);
    }
}

int main(void)
{
    int i;

    /* Make sure the vector table is relocated correctly (after the Tomu bootloader) */
    SCB_VTOR = 0x4000;

    /* Disable the watchdog that the bootloader started. */
    WDOG_CTRL = 0;

    /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set up both LEDs as outputs */
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

    /* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, hid_set_config);

    /* Enable USB IRQs */
    nvic_set_priority(NVIC_USB_IRQ, 0x40);
	nvic_enable_irq(NVIC_USB_IRQ);

    /* Configure the system tick, at lower priority than USB IRQ */
    systick_set_frequency(SYSTICK_FREQUENCY, AHB_FREQUENCY);
    systick_counter_enable();
    systick_interrupt_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0x10);

    while(1) {
        gpio_clear(LED_RED_PORT, LED_RED_PIN);
        for(i = 0; i != 500000; ++i)
			__asm__("nop");

    }
}

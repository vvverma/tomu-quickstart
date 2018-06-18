/* file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
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
 * This is small USB controlled LED blinking example program using libopencm3.
 * The usbtest.py script in this directory maybe used to control the LED-Color.
 * R- Red G-Green OFF (Both Leds OFF)
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY 14000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0101    /* Program version */

usbd_device *g_usbd_dev = 0;

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xFF,
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
static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
},{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};
const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,
	.endpoint = data_endp,
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

const char *usb_strings[] = {
	"Tomu",
	"Simple Device",
	"1001",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes simple_control_callback(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)buf;
	(void)len;
	(void)complete;
	(void)usbd_dev;

	if (req->bmRequestType != 0x40)
		return 0; /* Only accept vendor request. */
/*
	if (req->wValue & 1)
        gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
	else
        gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
*/
	return 1;
}
static void data_rx(usbd_device *usbd_dev, uint8_t ep){
(void)ep;

char buf[64];
int len;

    /* We got some data, toggle the green LED */

	len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if ((strncmp(buf,"G",1) == 0)) {
        gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
		gpio_set(LED_RED_PORT, LED_RED_PIN);
	}
	else if(( strncmp (buf,"R",1)) == 0 ) {
        gpio_clear(LED_RED_PORT, LED_RED_PIN);
		gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
	}
	else {
		gpio_set(LED_RED_PORT, LED_RED_PIN);
		gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
	}
	if (len) {
	 	usbd_ep_write_packet(usbd_dev, 0x82, buf,len);
	    buf[len] = 0;
	}
}
/* TODO
static void data_tx(usbd_device *usbd_dev, uint8_t ep){
	(void) ep;

}
*/

static void usb_set_config_cb(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, data_rx);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, 0);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR,
				USB_REQ_TYPE_TYPE,
				simple_control_callback);
}

void usb_isr(void)
{
    usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
    while(1);
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
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, usb_set_config_cb);

    /* Enable USB IRQs */
	nvic_enable_irq(NVIC_USB_IRQ);
	gpio_set(LED_RED_PORT, LED_RED_PIN);
	gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);

    while(1) {

        for(i = 0; i != 500000; ++i)
			__asm__("nop");
    }
}

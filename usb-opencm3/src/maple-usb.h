/*
 * maple-usb.h
 *
 *  Created on: Aug 21, 2016
 *      Author: lieven2
 */

#ifndef MAPLE_USB_H_
#define MAPLE_USB_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req,
		uint8_t **buf, uint16_t *len,
		void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req));
void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);
void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);
extern const struct usb_device_descriptor dev;
extern uint8_t usbd_control_buffer[128];
extern const char *usb_strings[];
extern const struct usb_config_descriptor config;

#ifdef __cplusplus
}
#endif




#endif /* MAPLE_USB_H_ */

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

/*- Includes ----------------------------------------------------------------*/
#include "usb_std.h"
#include "usb_cdc.h"
#include "usb_hid.h"
#include "usb_winusb.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_ENABLE_BOS
#define USB_BCD_VERSION      0x0210

enum
{
  USB_STR_ZERO,
  USB_STR_MANUFACTURER,
  USB_STR_PRODUCT,
  USB_STR_SERIAL_NUMBER,
  USB_STR_COUNT,
};

enum
{
  USB_BULK_EP_SEND = 1,
};

enum
{
  USB_INTF_BULK,
  USB_INTF_COUNT,
};

/*- Types -------------------------------------------------------------------*/
typedef struct USB_PACK
{
  usb_configuration_descriptor_t                   configuration;
  usb_interface_descriptor_t                       interface;
  usb_endpoint_descriptor_t                        ep_in;
} usb_configuration_hierarchy_t;

typedef struct
{
  usb_binary_object_store_descriptor_t             bos;
  usb_winusb_capability_descriptor_t               winusb;
} usb_bos_hierarchy_t;

typedef struct
{
  usb_winusb_set_header_descriptor_t               header;
  usb_winusb_feature_compatble_id_t                comp_id;
  usb_winusb_feature_reg_property_guids_t          property;
} usb_msos_descriptor_set_t;

/*- Variables ---------------------------------------------------------------*/
extern const usb_device_descriptor_t usb_device_descriptor;
extern const usb_configuration_hierarchy_t usb_configuration_hierarchy;
extern const usb_bos_hierarchy_t usb_bos_hierarchy;
extern const usb_msos_descriptor_set_t usb_msos_descriptor_set;
extern const usb_string_descriptor_zero_t usb_string_descriptor_zero;
extern const char *usb_strings[];
extern const usb_class_handler_t usb_class_handlers[2];
extern char usb_serial_number[16];

/*- Prototypes --------------------------------------------------------------*/
bool vendor_handle_request(usb_request_t *request);

#endif // _USB_DESCRIPTORS_H_



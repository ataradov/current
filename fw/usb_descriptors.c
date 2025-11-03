// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdalign.h>
#include "usb_descriptors.h"

/*- Variables ---------------------------------------------------------------*/
const alignas(4) usb_device_descriptor_t usb_device_descriptor =
{
  .bLength            = sizeof(usb_device_descriptor_t),
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR,
  .bcdUSB             = USB_BCD_VERSION,
  .bDeviceClass       = 0x00,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,
  .bMaxPacketSize0    = USB_CTRL_EP_SIZE,
  .idVendor           = 0x6666,
  .idProduct          = 0x1201,
  .bcdDevice          = 0x0100,
  .iManufacturer      = USB_STR_MANUFACTURER,
  .iProduct           = USB_STR_PRODUCT,
  .iSerialNumber      = USB_STR_SERIAL_NUMBER,
  .bNumConfigurations = 1,
};

const alignas(4) usb_configuration_hierarchy_t usb_configuration_hierarchy =
{
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = USB_INTF_COUNT,
    .bConfigurationValue = 1,
    .iConfiguration      = 0,
    .bmAttributes        = USB_ATTRIBUTE_BUS_POWERED,
    .bMaxPower           = USB_MAX_POWER(500),
  },

  .interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = USB_INTF_BULK,
    .bAlternateSetting   = 0,
    .bNumEndpoints       = 1,
    .bInterfaceClass     = USB_DEVICE_CLASS_VENDOR_SPECIFIC,
    .bInterfaceSubClass  = 0,
    .bInterfaceProtocol  = 0,
    .iInterface          = 0,
  },

  .ep_in =
  {
    .bLength             = sizeof(usb_endpoint_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = USB_IN_ENDPOINT | USB_BULK_EP_SEND,
    .bmAttributes        = USB_BULK_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 0,
  },
};

const alignas(4) usb_bos_hierarchy_t usb_bos_hierarchy =
{
  .bos =
  {
    .bLength             = sizeof(usb_binary_object_store_descriptor_t),
    .bDescriptorType     = USB_BINARY_OBJECT_STORE_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_bos_hierarchy_t),
    .bNumDeviceCaps      = 1,
  },

  .winusb =
  {
    .bLength                = sizeof(usb_winusb_capability_descriptor_t),
    .bDescriptorType        = USB_DEVICE_CAPABILITY_DESCRIPTOR,
    .bDevCapabilityType     = USB_DEVICE_CAPABILITY_PLATFORM,
    .bReserved              = 0,
    .PlatformCapabilityUUID = USB_WINUSB_PLATFORM_CAPABILITY_ID,
    .dwWindowsVersion       = USB_WINUSB_WINDOWS_VERSION,
    .wMSOSDescriptorSetTotalLength = sizeof(usb_msos_descriptor_set_t),
    .bMS_VendorCode         = USB_WINUSB_VENDOR_CODE,
    .bAltEnumCode           = 0,
  },
};

const alignas(4) usb_msos_descriptor_set_t usb_msos_descriptor_set =
{
  .header =
  {
    .wLength             = sizeof(usb_winusb_set_header_descriptor_t),
    .wDescriptorType     = USB_WINUSB_SET_HEADER_DESCRIPTOR,
    .dwWindowsVersion    = USB_WINUSB_WINDOWS_VERSION,
    .wDescriptorSetTotalLength = sizeof(usb_msos_descriptor_set_t),
  },

  .comp_id =
  {
    .wLength           = sizeof(usb_winusb_feature_compatble_id_t),
    .wDescriptorType   = USB_WINUSB_FEATURE_COMPATBLE_ID,
    .CompatibleID      = "WINUSB",
    .SubCompatibleID   = { 0 },
  },

  .property =
  {
    .wLength             = sizeof(usb_winusb_feature_reg_property_guids_t),
    .wDescriptorType     = USB_WINUSB_FEATURE_REG_PROPERTY,
    .wPropertyDataType   = USB_WINUSB_PROPERTY_DATA_TYPE_SZ,
    .wPropertyNameLength = sizeof(usb_msos_descriptor_set.property.PropertyName),
    .PropertyName = {
        'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,'r',0,'f',0,'a',0,'c',0,'e',0,
        'G',0,'U',0,'I',0,'D',0, 0, 0 },
    .wPropertyDataLength = sizeof(usb_msos_descriptor_set.property.PropertyData),
    .PropertyData = {
        '{',0,'8',0,'8',0,'B',0,'A',0,'E',0,'0',0,'3',0,'2',0,'-',0,'5',0,'A',0,'8',0,'1',0,'-',0,
        '4',0,'9',0,'f',0,'0',0,'-',0,'B',0,'C',0,'3',0,'D',0,'-',0,'A',0,'4',0,'F',0,'F',0,'1',0,
        '3',0,'8',0,'2',0,'1',0,'6',0,'D',0,'6',0,'}',0, 0, 0 },
  },
};

const alignas(4) usb_string_descriptor_zero_t usb_string_descriptor_zero =
{
  .bLength         = sizeof(usb_string_descriptor_zero_t),
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .wLANGID         = USB_LANGID_ENGLISH,
};

const char *usb_strings[] =
{
  [USB_STR_MANUFACTURER]  = "Alex Taradov",
  [USB_STR_PRODUCT]       = "Current Profiler",
  [USB_STR_SERIAL_NUMBER] = usb_serial_number,
};

const usb_class_handler_t usb_class_handlers[2] =
{
  vendor_handle_request,
  usb_winusb_handle_request,
};

char usb_serial_number[16];



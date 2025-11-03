// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _USB_H_
#define _USB_H_

/*- Includes ----------------------------------------------------------------*/
#include "os.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_BLOCK_SIZE       32

/*- Prototypes --------------------------------------------------------------*/
void usb_start(void);
void usb_stop(void);
void usb_cal_read(u8 *data, int size);
void usb_cal_write(u8 *data, int size);
bool usb_read_buf(u16 data[USB_BLOCK_SIZE]);
void usb_set_range(bool fixed, bool high);

#endif // _USB_H_



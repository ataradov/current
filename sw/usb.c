// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <libusb.h>
#include "os.h"
#include "os_thread.h"
#include "current.h"
#include "usb.h"

/*- Definitions -------------------------------------------------------------*/
#define CURRENT_VID            0x6666
#define CURRENT_PID            0x1201

#define DATA_ENDPOINT          0x81
#define DATA_ENDPOINT_SIZE     64
#define TRANSFER_SIZE          (DATA_ENDPOINT_SIZE * 128)
#define TRANSFER_COUNT         8
#define TRANSFER_TIMEOUT       1000 // ms

#define CTRL_ENABLE            (1 << 0)
#define CTRL_HIGH_RANGE        (1 << 1)
#define CTRL_FIXED_RANGE       (1 << 2)

#define USB_CMD_CTRL           0x55
#define USB_CMD_CAL            0x56

#define USB_BUFFER_SIZE        (16*1024*1024)

/*- Variables ---------------------------------------------------------------*/
static libusb_device_handle *usb_handle = NULL;
static struct libusb_transfer *usb_transfer[TRANSFER_COUNT];
static u8 *usb_transfer_buf[TRANSFER_COUNT];
static OsThread *usb_thread;
static bool usb_thread_run;
static OsMutex *usb_mutex;
static u8 usb_buffer[USB_BUFFER_SIZE];
static int usb_wr_ptr;
static int usb_rd_ptr;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void usb_check_error(int error, const char *text)
{
  if (error < 0)
    os_error("%s: %s\n", text, libusb_error_name(error));
}

//-----------------------------------------------------------------------------
void usb_cal_read(u8 *data, int size)
{
  int rc = libusb_control_transfer(usb_handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
    USB_CMD_CAL, 0/*wValue*/, 0/*wIndex*/, data, size, TRANSFER_TIMEOUT);

  usb_check_error(rc, "usb_cal_read()");
}

//-----------------------------------------------------------------------------
void usb_cal_write(u8 *data, int size)
{
  int rc = libusb_control_transfer(usb_handle,
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
    USB_CMD_CAL, 0/*wValue*/, 0/*wIndex*/, data, size, TRANSFER_TIMEOUT);

  usb_check_error(rc, "usb_cal_write()");
}

//-----------------------------------------------------------------------------
static void usb_ctrl(int value)
{
  int rc = libusb_control_transfer(usb_handle,
      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
      USB_CMD_CTRL, value/*wValue*/, 0/*wIndex*/, NULL, 0, TRANSFER_TIMEOUT);

  usb_check_error(rc, "usb_ctrl()");
}

//-----------------------------------------------------------------------------
bool usb_read_buf(u16 data[USB_BLOCK_SIZE])
{
  os_mutex_lock(usb_mutex);

  if (usb_wr_ptr == usb_rd_ptr)
  {
    os_mutex_unlock(usb_mutex);
    return false;
  }

  for (int i = 0; i < USB_BLOCK_SIZE; i++)
  {
    int h = usb_buffer[usb_rd_ptr + i*2];
    int l = usb_buffer[usb_rd_ptr + i*2 + 1];
    data[i] = (h << 8) | l;
  }

  usb_rd_ptr = (usb_rd_ptr + DATA_ENDPOINT_SIZE) % USB_BUFFER_SIZE;

  os_mutex_unlock(usb_mutex);

  os_check(data[0] > 0, "hardware buffer overflow");

  return true;
}

//-----------------------------------------------------------------------------
void usb_set_range(bool fixed, bool high)
{
  usb_ctrl(CTRL_ENABLE | (high ? CTRL_HIGH_RANGE : 0) | (fixed ? CTRL_FIXED_RANGE : 0));
}

//-----------------------------------------------------------------------------
static void LIBUSB_CALL usb_capture_callback(struct libusb_transfer *transfer)
{
  int rc;

  /*if (LIBUSB_TRANSFER_TIMED_OUT == transfer->status)
    {}
  else*/ if (LIBUSB_TRANSFER_COMPLETED != transfer->status)
    os_error("usb_capture_callback(): %d\n", transfer->status);

  os_check(0 == (transfer->actual_length % DATA_ENDPOINT_SIZE), "invalid data size");

  os_mutex_lock(usb_mutex);

  for (int i = 0; i < transfer->actual_length; i += DATA_ENDPOINT_SIZE)
  {
    memcpy(&usb_buffer[usb_wr_ptr], &transfer->buffer[i], DATA_ENDPOINT_SIZE);

    usb_wr_ptr = (usb_wr_ptr + DATA_ENDPOINT_SIZE) % USB_BUFFER_SIZE;

    os_check(usb_wr_ptr != usb_rd_ptr, "USB buffer overflow");
  }

  os_mutex_unlock(usb_mutex);

  rc = libusb_submit_transfer(transfer);
  usb_check_error(rc, "libusb_submit_transfer() in usb_capture_callback()");
}

//-----------------------------------------------------------------------------
static libusb_device_handle *usb_open(void)
{
  libusb_device **devices;
  libusb_device_handle *handle = NULL;
  int rc, count;

  count = libusb_get_device_list(NULL, &devices);
  usb_check_error(count, "libusb_get_device_list()");

  for (int i = 0; i < count; i++)
  {
    libusb_device *dev = devices[i];
    struct libusb_device_descriptor desc;

    rc = libusb_get_device_descriptor(dev, &desc);
    usb_check_error(rc, "libusb_get_device_descriptor()");

    if (CURRENT_VID == desc.idVendor && CURRENT_PID == desc.idProduct)
    {
      rc = libusb_open(dev, &handle);
      usb_check_error(rc, "libusb_open()");
      break;
    }
  }

  libusb_free_device_list(devices, 1);

  if (!handle)
    return NULL;

  libusb_set_auto_detach_kernel_driver(handle, 1);

  rc = libusb_claim_interface(handle, 0);
  usb_check_error(rc, "libusb_claim_interface()");

  return handle;
}

//-----------------------------------------------------------------------------
static void usb_flush_data(void)
{
  int rc, size;
  u8 buf[DATA_ENDPOINT_SIZE];

  for (int k = 0; k < 100; k++)
  {
    rc = libusb_bulk_transfer(usb_handle, DATA_ENDPOINT, buf, sizeof(buf), &size, 20);

    if (rc == LIBUSB_ERROR_TIMEOUT)
      break;
    else
      usb_check_error(rc, "libusb_bulk_transfer()");
  }
}

//-----------------------------------------------------------------------------
static void usb_setup_transfers(void)
{
  for (int i = 0; i < TRANSFER_COUNT; i++)
  {
    usb_transfer_buf[i]   = os_alloc(TRANSFER_SIZE);
    usb_transfer[i] = libusb_alloc_transfer(0);
    os_check(usb_transfer[i], "libusb_alloc_transfer()");

    libusb_fill_bulk_transfer(usb_transfer[i], usb_handle, DATA_ENDPOINT,
        usb_transfer_buf[i], TRANSFER_SIZE, usb_capture_callback, NULL, TRANSFER_TIMEOUT);

    int rc = libusb_submit_transfer(usb_transfer[i]);
    usb_check_error(rc, "libusb_submit_transfer()");
  }
}

//-----------------------------------------------------------------------------
void *usb_thread_handler(void *arg)
{
  usb_ctrl(0);
  usb_flush_data();
  usb_setup_transfers();

  usb_ctrl(CTRL_ENABLE | CTRL_HIGH_RANGE | CTRL_FIXED_RANGE);

  while (usb_thread_run)
  {
    libusb_handle_events(NULL);
  }

  return arg;
}

//-----------------------------------------------------------------------------
void usb_start(void)
{
  int rc = libusb_init(NULL);
  usb_check_error(rc, "libusb_init()");

  usb_handle = usb_open();

  if (!usb_handle)
    os_error("no capture hardware found");

  usb_mutex = os_mutex_create();

  usb_thread_run = true;
  usb_thread = os_thread_create(usb_thread_handler, NULL);
}

//-----------------------------------------------------------------------------
void usb_stop(void)
{
  usb_ctrl(0);

  usb_thread_run = false;

  libusb_close(usb_handle);

  os_thread_join(usb_thread);

  libusb_exit(NULL);
}



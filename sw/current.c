// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2025, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include "os.h"
#include "usb.h"
#include "gui.h"
#include "current.h"

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int main(void)
{
  usb_start();
  gui_run();
  usb_stop();

  return 0;
}



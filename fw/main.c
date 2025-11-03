// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd11.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "usb.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_EP_SIZE            64

#define USB_CMD_CTRL           0x55
#define USB_CMD_CAL            0x56

#define BUFFER_COUNT           16
#define BUFFER_SIZE            (USB_EP_SIZE / sizeof(uint16_t))
#define BUFFER_FULL_SIZE       (BUFFER_SIZE * BUFFER_COUNT)

#define GCLK_ID_SYS            0
#define GCLK_ID_DPLL_REF       1
#define GCLK_ID_DPLL_48M       2
#define GCLK_ID_DPLL_96M       3

HAL_GPIO_PIN(TRIG,             A, 2);
HAL_GPIO_PIN(LED,              A, 4);
HAL_GPIO_PIN(RANGE,            A, 5);
HAL_GPIO_PIN(SCLK,             A, 9);
HAL_GPIO_PIN(DOUT,             A, 14);
HAL_GPIO_PIN(CONVST,           A, 15);

#define CTRL_ENABLE            (1 << 0)
#define CTRL_RANGE             (1 << 1)
#define CTRL_FIXED_RANGE       (1 << 2)

#define CAL_ADDR               (FLASH_ADDR + FLASH_SIZE - FLASH_PAGE_SIZE * 4)

/*- Variables ---------------------------------------------------------------*/
static volatile DmacDescriptor dma_desc[DMAC_CH_NUM];
static volatile DmacDescriptor dma_wb[DMAC_CH_NUM];
static volatile alignas(4) uint16_t g_buf[BUFFER_FULL_SIZE];
static volatile int g_free_buf_count = BUFFER_COUNT;
static volatile int g_wr_ptr = 0;
static volatile int g_rd_ptr = 0;
static volatile uint16_t g_adc_value;
static volatile bool g_enabled = false;
static volatile bool g_usb_ready = false;
static volatile bool g_overflow = false;
static volatile int g_range = 0;
static volatile bool g_fixed_range = false;
static uint32_t dummy = 0xffffffff;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  uint32_t coarse, fine;

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS(1) | NVMCTRL_CTRLB_MANW;

  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
      SYSCTRL_INTFLAG_DFLLRDY;

  coarse = NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL);
  fine = NVM_READ_CAL(NVM_DFLL48M_FINE_CAL);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
      SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_CCDIS;

  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(GCLK_ID_SYS) | GCLK_GENCTRL_SRC_DFLL48M |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

//-----------------------------------------------------------------------------
static void serial_number_init(void)
{
  uint32_t wuid[4];
  uint8_t *uid = (uint8_t *)wuid;
  uint32_t sn = 5381;

  wuid[0] = *(volatile uint32_t *)0x0080a00c;
  wuid[1] = *(volatile uint32_t *)0x0080a040;
  wuid[2] = *(volatile uint32_t *)0x0080a044;
  wuid[3] = *(volatile uint32_t *)0x0080a048;

  for (int i = 0; i < 16; i++)
    sn = ((sn << 5) + sn) ^ uid[i];

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[8] = 0;
}

//-----------------------------------------------------------------------------
static void set_range(bool range)
{
  g_range = range << 8;
  HAL_GPIO_RANGE_write(range);
}

//-----------------------------------------------------------------------------
static void capture_start(void)
{
  HAL_GPIO_LED_set();
  g_enabled  = true;
  TCC0->INTFLAG.reg = TCC_INTFLAG_MASK;
  NVIC_EnableIRQ(TCC0_IRQn);
}

//-----------------------------------------------------------------------------
static void capture_stop(void)
{
  NVIC_DisableIRQ(TCC0_IRQn);
  HAL_GPIO_LED_clr();
  g_enabled = false;
}

//-----------------------------------------------------------------------------
static void cal_write_handler(uint8_t *data, int size)
{
  uint32_t *buf = (uint32_t *)data;

  NVMCTRL->ADDR.reg = CAL_ADDR >> 1;

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
  while (0 == NVMCTRL->INTFLAG.bit.READY);

  for (uint32_t i = 0; i < size / sizeof(uint32_t); i++)
    *(uint32_t *)(CAL_ADDR + i * sizeof(uint32_t)) = buf[i];

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
  while (0 == NVMCTRL->INTFLAG.bit.READY);
}

//-----------------------------------------------------------------------------
bool vendor_handle_request(usb_request_t *request)
{
  switch (USB_CMD_VALUE(request))
  {
    case USB_CMD(OUT, DEVICE, VENDOR, CMD_CTRL):
    {
      if (request->wValue & CTRL_ENABLE)
        capture_start();
      else
        capture_stop();

      g_fixed_range = (request->wValue & CTRL_FIXED_RANGE) > 0;

      if (g_fixed_range)
        set_range(request->wValue & CTRL_RANGE);

      usb_control_send_zlp();
    } break;

    case USB_CMD(IN, DEVICE, VENDOR, CMD_CAL):
    {
      usb_control_send((uint8_t *)CAL_ADDR, request->wLength);
    } break;

    case USB_CMD(OUT, DEVICE, VENDOR, CMD_CAL):
    {
      usb_control_recv(cal_write_handler);
    } break;

    default:
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
static void dpll_init(void)
{
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(GCLK_ID_DPLL_REF) | GCLK_GENDIV_DIV(48);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(GCLK_ID_DPLL_REF) | GCLK_GENCTRL_SRC_DFLL48M |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_FDPLL | GCLK_CLKCTRL_CLKEN |
      GCLK_CLKCTRL_GEN(GCLK_ID_DPLL_REF);

  SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_REFCLK_GCLK;
  SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDR(96 - 1);
  SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ENABLE | SYSCTRL_DPLLCTRLA_RUNSTDBY;

  while (0 == (SYSCTRL->DPLLSTATUS.reg & SYSCTRL_DPLLSTATUS_LOCK));
  while (0 == (SYSCTRL->DPLLSTATUS.reg & SYSCTRL_DPLLSTATUS_CLKRDY));

  GCLK->GENDIV.reg = GCLK_GENDIV_ID(GCLK_ID_DPLL_48M) | GCLK_GENDIV_DIV(2);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(GCLK_ID_DPLL_48M) | GCLK_GENCTRL_SRC_DPLL96M |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_IDC;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->GENDIV.reg = GCLK_GENDIV_ID(GCLK_ID_DPLL_96M) | GCLK_GENDIV_DIV(1);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(GCLK_ID_DPLL_96M) | GCLK_GENCTRL_SRC_DPLL96M |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_IDC;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

//-----------------------------------------------------------------------------
static void dma_init(void)
{
  DMAC->BASEADDR.reg = (uint32_t)dma_desc;
  DMAC->WRBADDR.reg  = (uint32_t)dma_wb;

  DMAC->PRICTRL0.reg = DMAC_PRICTRL0_RRLVLEN0 | DMAC_PRICTRL0_RRLVLEN1 |
      DMAC_PRICTRL0_RRLVLEN2 | DMAC_PRICTRL0_RRLVLEN3;

  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 |
      DMAC_CTRL_LVLEN2 | DMAC_CTRL_LVLEN3;

  // Receive channel
  DMAC->CHID.reg = 0;
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_RX);

  dma_desc[0].SRCADDR.reg  = (uint32_t)&SERCOM0->SPI.DATA.reg;
  dma_desc[0].DSTADDR.reg  = (uint32_t)&g_adc_value + sizeof(g_adc_value);
  dma_desc[0].BTCNT.reg    = sizeof(g_adc_value);
  dma_desc[0].BTCTRL.reg   = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_DSTINC;
  dma_desc[0].DESCADDR.reg = (uint32_t)&dma_desc[0];

  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE;

  // Transmit channel
  DMAC->CHID.reg = 1;
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_TX) |
      DMAC_CHCTRLB_EVACT_CBLOCK | DMAC_CHCTRLB_EVIE;

  dma_desc[1].SRCADDR.reg  = (uint32_t)&dummy;
  dma_desc[1].DSTADDR.reg  = (uint32_t)&SERCOM0->SPI.DATA.reg;
  dma_desc[1].BTCNT.reg    = sizeof(g_adc_value);
  dma_desc[1].BTCTRL.reg   = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BEATSIZE_BYTE;
  dma_desc[1].DESCADDR.reg = (uint32_t)&dma_desc[1];

  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE;
}

//-----------------------------------------------------------------------------
static void evsys_init(void)
{
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EVSYS_0 | GCLK_CLKCTRL_CLKEN |
      GCLK_CLKCTRL_GEN(GCLK_ID_SYS);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EVSYS_1 | GCLK_CLKCTRL_CLKEN |
      GCLK_CLKCTRL_GEN(GCLK_ID_SYS);

  EVSYS->CTRL.reg = EVSYS_CTRL_GCLKREQ;

  EVSYS->USER.reg = EVSYS_USER_USER(0x01/*DMAC CH1*/) | EVSYS_USER_CHANNEL(0+1);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0) | EVSYS_CHANNEL_PATH_RESYNCHRONIZED |
      EVSYS_CHANNEL_EDGSEL_RISING_EDGE | EVSYS_CHANNEL_EVGEN(0x1b/*TCC0_MCX0*/);
}

//-----------------------------------------------------------------------------
static void adc_init(void)
{
  HAL_GPIO_SCLK_out();
  HAL_GPIO_SCLK_pmuxen(PORT_PMUX_PMUXE_D_Val);

  HAL_GPIO_DOUT_in();
  HAL_GPIO_DOUT_pmuxen(PORT_PMUX_PMUXE_C_Val);

  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(GCLK_ID_DPLL_48M);

  SERCOM0->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;

  SERCOM0->SPI.BAUD.reg = 1; // 12 MHz

  SERCOM0->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_ENABLE | SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
      SERCOM_SPI_CTRLA_DIPO(0) | SERCOM_SPI_CTRLA_DOPO(1);
}

//-----------------------------------------------------------------------------
static void tcc_init(void)
{
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC0 | GCLK_CLKCTRL_CLKEN |
      GCLK_CLKCTRL_GEN(GCLK_ID_DPLL_96M);

  HAL_GPIO_CONVST_out();
  HAL_GPIO_CONVST_pmuxen(PORT_PMUX_PMUXE_F_Val);

  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER(0) | TCC_CTRLA_PRESCSYNC_PRESC;
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;

/*
  TCC0->PER.reg   = 383;
  TCC0->CC[0].reg = 90;  // DMA Start
  TCC0->CC[1].reg = 130; // CONVST output
  TCC0->CC[2].reg = 220; // DMA data ready
*/

  TCC0->PER.reg   = 383;
  TCC0->CC[0].reg = 160; // DMA Start
  TCC0->CC[1].reg = 180; // CONVST output
  TCC0->CC[2].reg = 340; // DMA data ready

  TCC0->EVCTRL.reg = TCC_EVCTRL_MCEO0;
  TCC0->INTENSET.reg = TCC_INTFLAG_MC2;

  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
}

//-----------------------------------------------------------------------------
__attribute__((section(".ramfunc")))
void irq_handler_tcc0(void)
{
  int value = (g_adc_value & 0xfeff) | g_range;

  TCC0->INTFLAG.reg = TCC_INTFLAG_MASK;

  if (g_free_buf_count > 0)
  {
    g_buf[g_wr_ptr++] = value;

    g_wr_ptr %= BUFFER_FULL_SIZE;

    if (0 == (g_wr_ptr % BUFFER_SIZE))
      g_free_buf_count--;
  }
  else
  {
    g_overflow = true;
  }
}

//-----------------------------------------------------------------------------
static void usb_bulk_send_callback(void)
{
  NVIC_DisableIRQ(TCC0_IRQn);

  g_free_buf_count++;

  if (g_enabled)
    NVIC_EnableIRQ(TCC0_IRQn);

  g_rd_ptr = (g_rd_ptr + BUFFER_SIZE) % BUFFER_FULL_SIZE;

  g_usb_ready = true;
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  usb_set_send_callback(USB_BULK_EP_SEND, usb_bulk_send_callback);
  g_usb_ready = true;
  (void)config;
}

//-----------------------------------------------------------------------------
static void tx_task(void)
{
  if (!g_usb_ready || BUFFER_COUNT == g_free_buf_count)
    return;

  // g_buf[g_rd_ptr] = (g_buf[g_rd_ptr] & 0xfdff) | (HAL_GPIO_TRIG_read() << 9);

  if (!g_fixed_range)
  {
    int value = __REV(g_buf[g_rd_ptr] << 16);

    if (g_range)
    {
      if (value < 7000)
        set_range(false);
    }
    else
    {
      if (value > 63000)
        set_range(true);
    }
  }

  if (g_overflow)
  {
    g_buf[g_rd_ptr] = 0;
    g_overflow = false;
  }

  g_usb_ready = false;
  usb_send(USB_BULK_EP_SEND, (uint8_t *)&g_buf[g_rd_ptr], USB_EP_SIZE);
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  usb_init();
  serial_number_init();
  dpll_init();
  adc_init();
  evsys_init();
  dma_init();
  tcc_init();

  HAL_GPIO_RANGE_out();
  set_range(true);

  HAL_GPIO_LED_out();
  HAL_GPIO_LED_clr();

  HAL_GPIO_TRIG_in();
  HAL_GPIO_TRIG_pullup();

  while (1)
  {
    usb_task();
    tx_task();

#if 1
    if (0 == HAL_GPIO_TRIG_read())
      NVIC_SystemReset();
#endif
  }

  return 0;
}



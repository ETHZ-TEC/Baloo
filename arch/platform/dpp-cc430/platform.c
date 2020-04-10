/*
* Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/platform.h"
#include "random.h"
#include "serial-line.h"
#include "node-id.h"
#include "energest.h"

#include "uart.h"
#include "rf1a.h"
#include "gpio.h"
#include "soc.h"
#include "watchdog.h"
#include "rtimer-ext.h"

#include "debug-print.h"
#include "dc-stat.h"

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "CC430"
#define LOG_LEVEL LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/
static void init_gpio(void);
static void print_device_info(void);
/*---------------------------------------------------------------------------*/
static uint16_t rst_flag;
extern uint16_t FLOCKLAB_NODE_ID;
/*---------------------------------------------------------------------------*/
void
platform_init_stage_one()
{
  watchdog_stop();

  init_gpio();
  soc_init();
  uart_init();
  uart_enable(1);
  rtimer_ext_init();
#if UART_CONF_RX_INTERRUPT
  uart_set_input_handler(serial_line_input_byte);
#endif /* UART_CONF_RX_INTERRUPT */
  print_device_info();
#if RF_CONF_ON
  /* init the radio module and set the parameters */
  rf1a_init();
#endif /* RF_CONF_ON */

#if SVS_CONF_ON
  SVS_ENABLE;
#else
  SVS_DISABLE;
#endif /* SVS_CONF_ON */
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_two()
{
#ifndef NODE_ID
  node_id = FLOCKLAB_NODE_ID;
#else
  node_id = NODE_ID;
#endif /* NODE_ID */
  random_init(node_id + TA0R);
#if UART_CONF_RX_INTERRUPT
  serial_line_init();
#endif /* UART_CONF_RX_INTERRUPT */
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_three()
{
  PIN_CLR(LED_STATUS);
  LOG_INFO("Node started. Node ID is set to %u. Platform " DPP_PLATFORM_STR
           "\n", node_id);
}
/*---------------------------------------------------------------------------*/
void
platform_idle()
{
  if(process_nevents() != 0 || UART_ACTIVE) {
    /* re-enable interrupts */
    __eint();
    __nop();
  } else {
    /* re-enable interrupts and go to sleep atomically */
    ENERGEST_OFF(ENERGEST_TYPE_CPU);
    DCSTAT_CPU_OFF;
    watchdog_stop();

    /* enter LPM3 */
    __eint();
    LPM3;
    __no_operation();

    watchdog_start();
    DCSTAT_CPU_ON;
    ENERGEST_ON(ENERGEST_TYPE_CPU);
  }
}
/*---------------------------------------------------------------------------*/
static void
init_gpio(void)
{
  /* set default configuration for all GPIOs */
  PORT_CLR_I(1);
  P1DIR = 0xff & ~(BIT1 | BIT5 | BIT6);
  PORT_CLR_I(2);
  P2DIR = 0xff & ~(BIT1 | BIT2 | BIT5);
  PORT_CLR_I(3);
  PORT_CFG_OUT_I(3);
  PORT_CLR_I(J);
  PORT_CFG_OUT_I(J);

  /* enable status LED to indicate start of init routine */
  PIN_SET(LED_STATUS);

#ifdef MUX_SEL_PIN
  /* this board has a multiplexer (set it to UART) */
  PIN_CFG_OUT(MUX_SEL_PIN);
  PIN_SET(MUX_SEL_PIN);
#endif

  /* pin mappings */
#ifdef RF_GDO0_PIN
  PIN_MAP_AS_OUTPUT(RF_GDO0_PIN, PM_RFGDO0);
#endif
#ifdef RF_GDO1_PIN
  PIN_MAP_AS_OUTPUT(RF_GDO1_PIN, PM_RFGDO1);
#endif
#ifdef RF_GDO2_PIN
  PIN_MAP_AS_OUTPUT(RF_GDO2_PIN, PM_RFGDO2);
#endif
#ifdef MCLK_PIN
  PIN_MAP_AS_OUTPUT(MCLK_PIN, PM_MCLK);
#endif
#ifdef SMCLK_PIN
  PIN_MAP_AS_OUTPUT(SMCLK_PIN, PM_SMCLK);
#endif
#ifdef ACLK_PIN
  PIN_MAP_AS_OUTPUT(ACLK_PIN, PM_ACLK);
#endif
#ifdef FLOCKLAB
  FLOCKLAB_INIT();
#endif /* FLOCKLAB */
}
/*---------------------------------------------------------------------------*/
/* prints some info about the system (e.g. MCU and reset source) */
static void
print_device_info(void)
{
  const char* rst_source[14] = { "BOR", "nRST", "SWBOR", "SECV", "SVS", "SVM",
      "SWPOR", "WDT", "WDTPW", "KEYV", "PLLUL", "PERF", "PMMKEY", "Unknown" };
  uint8_t idx;
  /*
   * note: this device does not offer an LPMx.5 mode, therefore there's no
   * corresponding reset source
   */
  rst_flag = SYSRSTIV; /* flag is automatically cleared by reading it */
  /* when the PMM causes a reset, a value is generated in the system reset
   interrupt vector generator register (SYSRSTIV), corresponding to the
   source of the reset */
  switch (rst_flag) {
    case SYSRSTIV_BOR:
      idx = 0;
      break;
    case SYSRSTIV_RSTNMI:
      idx = 1;
      break;
    case SYSRSTIV_DOBOR:
      idx = 2;
      break;
    case SYSRSTIV_SECYV:
      idx = 3;
      break;
    case SYSRSTIV_SVSL:
    case SYSRSTIV_SVSH:
      idx = 4;
      break;
    case SYSRSTIV_SVML_OVP:
    case SYSRSTIV_SVMH_OVP:
      idx = 5;
      break;
    case SYSRSTIV_DOPOR:
      idx = 6;
      break;
    case SYSRSTIV_WDTTO:
      idx = 7;
      break;
    case SYSRSTIV_WDTKEY:
      idx = 8;
      break;
    case SYSRSTIV_KEYV:
      idx = 9;
      break; /* flash password violation */
    case SYSRSTIV_PLLUL:
      idx = 10;
      break;
    case SYSRSTIV_PERF:
      idx = 11;
      break;
    case SYSRSTIV_PMMKEY:
      idx = 12;
      break;
    default:
      idx = 13;
      break;
  }
  LOG_INFO("MCU: " MCU_DESC ", Reset Source: %s, Compile date: " __DATE__ "\n",
           rst_source[idx]);

  /* note: KEYV indicates an incorrect FCTLx password was written to any flash
   * control register and generates a PUC when set. */
}
/*---------------------------------------------------------------------------*/

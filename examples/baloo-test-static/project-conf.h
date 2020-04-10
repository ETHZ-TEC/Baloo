/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Jonas Bächli
 *          Reto Da Forno
 *          Romain Jacob
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * application specific config file to override default settings
 */

#define HOST_ID                         5

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */
#ifdef FLOCKLAB
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
//  #define GMW_CONF_TASK_ACT_PIN         FLOCKLAB_INT2
//  #define RF_GDO2_PIN                   FLOCKLAB_INT1
//  #define DEBUG_PRINT_CONF_TASK_ACT_PIN FLOCKLAB_INT2
//  #define APP_TASK_ACT_PIN              FLOCKLAB_INT2
  /* note: FLOCKLAB_LED2 should not be used */
  #define DCSTAT_CPU_ON_PIN             FLOCKLAB_INT1
  #define DCSTAT_RF_ON_PIN              FLOCKLAB_INT2
//#define GMW_CONF_SLOT_ACT_PIN           FLOCKLAB_INT2
#else
  //#define GMW_CONF_SLOT_ACT_PIN         COM_GPIO3
#endif /* FLOCKLAB */

/* Platform-specific configuration */
#ifdef PLATFORM_SKY
  /* GPIO */
  #define GLOSSY_START_PIN              ADC0
  //#define GLOSSY_RF_PIN                 ADC0
  #define GLOSSY_TX_PIN                 ADC1
  #define GLOSSY_RX_PIN                 ADC2
  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_2405_MHz
  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

#elif defined PLATFORM_DPP_CC430
  /* GPIO */
  #define GLOSSY_START_PIN              COM_GPIO1
  //#define GLOSSY_RF_PIN                 COM_GPIO1
  #define GLOSSY_RX_PIN                 COM_GPIO2
  #define GLOSSY_TX_PIN                 COM_GPIO3
  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_869_0_MHz

#else  /* PLATFORM_ */
  #error "unknown target platform!"
#endif /* PLATFORM_ */

/* GMW configuration */
#define GMW_CONF_MAX_DATA_PKT_LEN       16
#define GMW_CONF_MAX_SLOTS              8
#define GMW_CONF_TX_CNT_DATA            2

#define GMW_CONF_PERIOD_TIME_BASE       GMW_CONF_PERIOD_TIME_BASE_1ms
#define GMW_CONF_T_GAP_CONTROL          15000
#define GMW_CONF_T_GAP                  12000

#define GMW_CONF_USE_STATIC_SCHED       1
#define GMW_CONF_USE_STATIC_CONFIG      1
#define GMW_CONF_CONTROL_USER_BYTES     1
#define GMW_CONF_USE_AUTOCLEAN          1

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_STACK_GUARD    (SRAM_START + SRAM_SIZE - 0x0200)

#endif /* __CONFIG_H__ */

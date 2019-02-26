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
 * Author:  Reto Da Forno
 *          Jonas Bächli
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * application specific config file to override default settings
 */

// #define FLOCKLAB                          /* uncomment to run on FlockLAB */
#define HOST_ID                         1
#define NUM_NODES                       30
#define SOURCE_IPI                      4    /* seconds */

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */
#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"
  /* set the highest antenna gain if the program runs on FlockLAB */
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #ifdef PLATFORM_SKY
    #define GLOSSY_TX_PIN               FLOCKLAB_LED2
    #define GLOSSY_RX_PIN               FLOCKLAB_LED3
  #else  /* PLATFORM_SKY */
    #define RF_GDO2_PIN                 FLOCKLAB_INT1
  #endif /* PLATFORM_SKY */
  #define GMW_CONF_TASK_ACT_PIN         FLOCKLAB_INT2
  #define DEBUG_PRINT_CONF_TASK_ACT_PIN FLOCKLAB_INT2
  #define APP_TASK_ACT_PIN              FLOCKLAB_INT2
#endif /* FLOCKLAB */

/* platform specific config */
#ifdef PLATFORM_SKY
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    //#define GLOSSY_RX_PIN               ADC2
    //#define GMW_CONF_TASK_ACT_PIN       ADC2
    #define DEBUG_PRINT_CONF_TASK_ACT_PIN ADC2
    //#define APP_TASK_ACT_PIN            ADC2
  #endif /* FLOCKLAB */
  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_2405_MHz
  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL
  /* stats */
  #define DCSTAT_CONF_ON                1

#elif defined PLATFORM_DPP_CC430
  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            COM_GPIO1
    #define RF_GDO2_PIN                 COM_GPIO2
    #define GMW_CONF_TASK_ACT_PIN       COM_GPIO3
    #define DEBUG_PRINT_CONF_TASK_ACT_PIN COM_GPIO3
    #define APP_TASK_ACT_PIN            COM_GPIO3
    /* lower TX power when running tests on the desk */
    #define GMW_CONF_RF_TX_POWER        GMW_RF_TX_POWER_MINUS_12_dBm
  #else /* FLOCKLAB */
    #define GMW_CONF_RF_TX_POWER        GMW_RF_TX_POWER_PLUS_10_dBm
  #endif /* FLOCKLAB */
  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_869_0_MHz
  /* stats */
  #define DCSTAT_CONF_ON                1

#else
  #error "unknown target platform"
#endif

/* LWB configuration */
#define LWB_SCHED_MIN_ENERGY             /* use the minimum energy scheduler */
#define LWB_CONF_SCHED_PERIOD_IDLE      5        /* define the period length */
#define LWB_CONF_SCHED_PERIOD_MIN       2
#define LWB_CONF_SCHED_PERIOD_MAX       15
#define LWB_CONF_OUT_BUFFER_SIZE        4
#define LWB_CONF_IN_BUFFER_SIZE         NUM_NODES
#define LWB_CONF_MAX_PKT_LEN            80
#define LWB_CONF_MAX_DATA_PKT_LEN       15
#define LWB_CONF_MAX_DATA_SLOTS         NUM_NODES

#define LWB_CONF_T_SCHED                (RTIMER_EXT_SECOND_HF / 40)     /* 25ms */
#define LWB_CONF_T_DATA                 (RTIMER_EXT_SECOND_HF / 50)     /* 20ms */
#define LWB_CONF_T_GUARD_1              (RTIMER_EXT_SECOND_HF / 1000)   /* 1ms */
#define LWB_CONF_T_GUARD_2              (RTIMER_EXT_SECOND_HF / 1000)
#define LWB_CONF_T_GUARD_3              (RTIMER_EXT_SECOND_HF / 1000)
#define LWB_CONF_T_GUARD                (RTIMER_EXT_SECOND_HF / 2000)   /* 0.5ms */
#define LWB_CONF_T_GAP                  (RTIMER_EXT_SECOND_HF / 200)    /* 5ms */
#define LWB_CONF_T_CONT                 (RTIMER_EXT_SECOND_HF / 125)    /* 8ms */
#define LWB_CONF_TX_CNT_SCHED           3
#define LWB_CONF_TX_CNT_DATA            3
#define LWB_CONF_T_SCHED2_START         RTIMER_EXT_SECOND_HF
/* LWB requires rtimer_ext_hf */
#define RTIMER_EXT_CONF_HF_ENABLE       1

/* Debug */
#define DEBUG_PRINT_CONF_STACK_GUARD    (SRAM_START + SRAM_SIZE - 0x0200)

#endif /* PROJECT_CONF_H_ */

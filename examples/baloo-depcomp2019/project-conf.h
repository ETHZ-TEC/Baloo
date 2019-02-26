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
 * Author:  Jonas Bächli
 *          Romain Jacob
 *          Reto Da Forno
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_


/*
 * application specific config file to overwrite default settings
 */

/* --- definitions for FLOCKLAB --- */
#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #define GLOSSY_TX_PIN                 FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                 FLOCKLAB_INT2
#endif /* FLOCKLAB */

/* --- definitions for GRAZ --- */
#ifdef GRAZ
  #include "../../tools/competition/graz.h"
  //#define GLOSSY_START_PIN              GPIO17
  //#define GLOSSY_TX_PIN                 GPIO4
  //#define GLOSSY_RX_PIN                 GPIO18
  #define EEPROM_PIN                    GPIO25
  //#define ACT_PIN                       GPIO4
#endif /* GRAZ */
/* Mapping GPIO-bits for visualization in Grafana
  # GPIO17   // 0x80
  # GPIO4    // 0x40
  # GPIO18   // 0x20
  # GPIO27   // 0x10
  # GPIO22   // 0x08
  # GPIO23   // 0x04
  # GPIO24   // 0x02
  # GPIO25   // 0x01
 */

/* --- PLATFORM dependent definitions --- */

#ifndef PLATFORM_SKY
  #error "target platform should be sky..."
#endif

/* GPIO config */
#if !defined FLOCKLAB && !defined GRAZ
  #define GLOSSY_START_PIN            ADC0
  #define GLOSSY_TX_PIN               ADC1
  #define GLOSSY_RX_PIN               ADC2
#endif /* FLOCKLAB */
/* RF - defined in the Makefile */
//#define GMW_CONF_RF_TX_CHANNEL        15
/* cc2420 specific config, don't change! */
#define CC2420_CONF_AUTOACK           0
#define CC2420_CONF_ADDRDECODE        0
#define CC2420_CONF_SFD_TIMESTAMPS    0
/* CPU frequency, don't change! */
#define F_CPU                         4194304UL
/* TX power setting */
#define GMW_CONF_RF_TX_POWER            GMW_RF_TX_PWR_0_dBm


/* --- GENERAL definitions --- */
#ifdef FLOCKLAB
#elif defined GRAZ
#else
#endif

/* Parse configuration */
#define HOST_ID                         cfg.p[0].destination_id[0]
#define PERIOD_S                        MAX((cfg.p[0].periodicity)/1000LU, 5)
#define MSG_LENGTH                      cfg.p[0].msg_length

/* GMW configuration */
#define GMW_CONF_T_DATA                 GMW_T_SLOT_MIN(\
                                          MSG_LENGTH \
                                          + GMW_CONF_RF_OVERHEAD, \
                                          GMW_CONF_TX_CNT_DATA, \
                                          GMW_CONF_MAX_HOPS)
#define GMW_CONF_TX_CNT_DATA            4
#define GMW_CONF_MAX_HOPS               5
// We write packets at teh destination in the slot_post_cb
// -> Make sure gap is big enough such that observer has time to read
//    data between to write operations.
#define T_EEPROM_WRITE                  5000  // us, arbitrary value for now
#define T_EEPROM_READ                   10    // ms, arbitrary value for now
#define GMW_CONF_T_GAP                  MAX( (20000LU - GMW_CONF_T_DATA), \
                                             T_EEPROM_WRITE)
//#define GMW_CONF_T_GAP_CONTROL          50000

//#define GMW_CONF_MAX_DATA_PKT_LEN       cfg.p[0].msg_length
// This does not work nice because the define is used to initialize array in the program
// -> Needs a constant
#define GMW_CONF_MAX_DATA_PKT_LEN       64
#define GMW_CONF_MAX_SLOTS              8
#define GMW_CONF_T_PREPROCESS           T_EEPROM_READ
#define GMW_CONF_USE_STATIC_CONFIG      1
#define GMW_CONF_USE_STATIC_SCHED       1

// We want to have at least one byte in the control packet payload
// otherwise Glossy does not work properly
#define GMW_CONF_USE_MAGIC_NUMBER       1


/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_PRINT_DBGLEVEL 1


#endif /* PROJECT_CONF_H_ */

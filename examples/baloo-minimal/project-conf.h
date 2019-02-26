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

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */
#ifdef FLOCKLAB

  #include "../../tools/flocklab/flocklab.h"
  
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #define GLOSSY_TX_PIN                 FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                 FLOCKLAB_INT2
#endif /* FLOCKLAB */


/* --- PLATFORM dependent definitions --- */

#ifdef PLATFORM_SKY
  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    #define GLOSSY_RX_PIN               ADC2
    //#define GMW_CONF_DEBUG_PIN          ADC7
  #endif /* FLOCKLAB */
  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_2405_MHz
  /* cc2420 specific config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

#elif defined PLATFORM_DPP_CC430
  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            COM_GPIO1
    //#define RF_GDO2_PIN                 COM_GPIO2
    #define GLOSSY_RX_PIN               COM_GPIO2
    #define GLOSSY_TX_PIN               COM_GPIO3
    /* lower TX power when running tests on the desk */
    #define GMW_CONF_RF_TX_POWER        GMW_RF_TX_POWER_MINUS_30_dBm
  #endif /* FLOCKLAB */
  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_868_6_MHz

#else
  #error "unknown target platform"
#endif


/* --- GENERAL definitions --- */

#define HOST_ID                         1

/* GMW configuration */
#define GMW_CONF_MAX_DATA_PKT_LEN       2
#define GMW_CONF_MAX_SLOTS              26

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_PRINT_DBGLEVEL 1

/* energy optimization */
#define UART1_CONF_RX_WITH_DMA          0   /* DMA requires DCO, hence cannot enter LPM3 -> disable DMA */
#define RTIMER_EXT_CONF_USE_ETIMER      0


#endif /* PROJECT_CONF_H_ */

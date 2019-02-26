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
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * application specific config file to override default settings
 */

#define SENDER_ID                       100

/* to compile for flocklab, uncomment the following define or pass FLOCKLAB=1
 * to the make command */
//#define FLOCKLAB
#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"
  #define RADIO_START_PIN               FLOCKLAB_LED1
  #define RADIO_TX_PIN                  FLOCKLAB_INT1
  #define RADIO_RX_PIN                  FLOCKLAB_INT2
#endif /* FLOCKLAB */


#define RADIO_CONF_PAYLOAD_LEN          20


/* --- PLATFORM dependent definitions --- */

#ifdef PLATFORM_SKY
  /* GPIO config */
  #ifndef FLOCKLAB
    #define RADIO_START_PIN             ADC0
    #define RADIO_TX_PIN                ADC1
    #define RADIO_RX_PIN                ADC2
  #endif /* FLOCKLAB */
  #define LED_STATUS                    LED2

  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  #define RF_CHANNEL                    15

#elif defined PLATFORM_DPP_CC430
  /* GPIO & RF config */
  #ifndef FLOCKLAB
    #define RADIO_START_PIN             COM_GPIO1
    #define RADIO_RX_PIN                COM_GPIO2
    #define RADIO_TX_PIN                COM_GPIO3
    /* cc430 radio config */
    #define RF_CHANNEL                  5                 /* approx. 869 MHz */
    #define RF_CONF_TX_POWER            RF1A_TX_POWER_MINUS_12_dBm
  #else
    /* cc430 radio config on Flocklab */
    #define RF_CHANNEL                  8               /* approx. 869.6 MHz */
    #define RF_CONF_TX_POWER            RF1A_TX_POWER_0_dBm
  #endif /* FLOCKLAB */
  /* for some reason, GDO2 pin has to be defined */
  //#define RF_GDO2_PIN                   PORT1, PIN0     //COM_GPIO3

#endif


/* general RF config */
#define RF_CONF_MAX_PKT_LEN             (RADIO_CONF_PAYLOAD_LEN + 10)
#define RF_CONF_TX_CH                   RF_CHANNEL


#endif /* PROJECT_CONF_H_ */

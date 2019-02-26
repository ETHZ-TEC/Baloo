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
 * Author:  Romain Jacob
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* Max size of the generated packet buffer */
#define PACKET_QUEUE_SIZE   10

/* --- PLATFORM dependent definitions --- */

#ifdef PLATFORM_DPP_CC430
// Compile for DPP
  #ifdef FLOCKLAB
    #include "../../tools/flocklab/flocklab.h"
    #define GLOSSY_START_PIN             FLOCKLAB_LED1
    #define GLOSSY_TX_PIN                FLOCKLAB_INT1
    #define GLOSSY_RX_PIN                FLOCKLAB_INT2
  #else
    //#define DCSTAT_RF_ON_PIN            COM_GPIO1
    #define GLOSSY_START_PIN             COM_GPIO1
    #define GLOSSY_TX_PIN                COM_GPIO2
    #define GLOSSY_RX_PIN                COM_GPIO3
    //#define LED_STATUS                  COM_GPIO3
  #endif /* FLOCKLAB */

  #define RF_CHANNEL                    5 /* approx. 869 MHz */

#elif defined PLATFORM_SKY
  #ifdef FLOCKLAB
    #include "../../tools/flocklab/flocklab.h"
    #define RADIO_START_PIN             FLOCKLAB_LED1
    //#define LED_STATUS                  FLOCKLAB_LED1
    #define RADIO_TX_PIN                FLOCKLAB_INT1
    #define RADIO_RX_PIN                FLOCKLAB_INT2
  #else
	  // The platform is set automatically to SKY when COOJA flag is set
	  #define RADIO_TX_PIN                  LED_RED
	  #define RADIO_RX_PIN                  LED_BLUE
	  //#define LED_STATUS                    LED_GREEN
    #define RADIO_START_PIN               LED_RED
  #endif /* FLOCKLAB */

  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  #define RF_CHANNEL                    12

#endif /* PLATFORM_DPP_CC430 */

/* application configuration */
#define DCSTAT_CONF_ON                  1

/* general RF config */
//#define RF_CONF_MAX_PKT_LEN             (RADIO_CONF_PAYLOAD_LEN + 10)
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm
#define RF_CONF_TX_CH                   RF_CHANNEL

/* GMW configuration */
#define HOST_ID                         22
#define GMW_CONF_MAX_SLOTS              14
#define GMW_CONF_PERIOD_TIME_BASE       GMW_CONF_PERIOD_TIME_BASE_1ms
#define GMW_CONF_T_GAP                  2000LU

#define GMW_CONF_MAX_DATA_PKT_LEN       5



#endif /* PROJECT_CONF_H_ */

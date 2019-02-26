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
 * Author:  Jonas Baechli
 *          Reto Da Forno
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * application specific config file to override default settings
 */

#define HOST_ID                         1

/* to compile for flocklab, uncomment the following define or pass FLOCKLAB=1
 * to the make command */
//#define FLOCKLAB
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
  #endif /* FLOCKLAB */

  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  #define RF_CHANNEL                    15
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

#elif defined PLATFORM_DPP_CC430
  /* cc430 radio config */
  #define RF_CHANNEL                    5                 /* approx. 869 MHz */
  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            COM_GPIO1
    #define GLOSSY_RX_PIN               COM_GPIO2
    #define GLOSSY_TX_PIN               COM_GPIO3
  #endif /* FLOCKLAB */

#endif


/* Glossy and general RF config */
#define RF_CONF_MAX_PKT_LEN             64
#define RF_CONF_TX_CH                   RF_CHANNEL
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm
#define GLOSSY_REF_OFS                  (RTIMER_EXT_SECOND_LF / 780)
#define GLOSSY_PERIOD                   (RTIMER_EXT_SECOND_LF / 10) /* 100ms */
#define GLOSSY_RTIMER_ID                RTIMER_EXT_LF_0
#define GLOSSY_T_SLOT                   (RTIMER_EXT_SECOND_LF / 50)  /* 20ms */
#define GLOSSY_T_GUARD                  (RTIMER_EXT_SECOND_LF / 2000) /*0.5ms*/
#define GLOSSY_PAYLOAD_LEN              8
#define GLOSSY_N_TX                     3

#endif /* PROJECT_CONF_H_ */

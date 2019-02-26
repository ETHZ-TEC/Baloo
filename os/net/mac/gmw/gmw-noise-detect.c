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
 * \author
 *         Reto Da Forno   rdaforno@ee.ethz.ch
 *         Romain Jacob    jacobr@ethz.ch
 */

/*---------------------------------------------------------------------------*/
/**
 * @addtogroup  gmw
 * @{
 */
/*---------------------------------------------------------------------------*/

/**
 * \file
 *            Implementation of the noise detection process for Baloo.
 *
 *            Part of the implementation is platform-dependent, as it builds
 *            directly upon radio functions.
 *
 * \note      Currently supported platforms are:
 *            - TelosB
 *            - DPP-CC430
 */

#include "contiki.h"
#include "gmw.h"
#include "debug-print.h"
#include "gpio.h"

#if GMW_CONF_USE_NOISE_DETECTION

#ifdef PLATFORM_DPP_CC430
static int16_t rssi_value;
static int16_t rssi_value_high_noise;
static int16_t rssi_value_last;
#endif /* PLATFORM_DPP_CC430 */

static uint16_t counter;
static uint16_t counter_high_noise;

/* pin used only for calibration purposes */
#ifdef GMW_NOISE_DETECT_PIN
#define NOISE_DETECT_ON          PIN_SET(GMW_NOISE_DETECT_PIN)
#define NOISE_DETECT_OFF         PIN_CLR(GMW_NOISE_DETECT_PIN)
#else
#define NOISE_DETECT_ON
#define NOISE_DETECT_OFF
#endif

#ifdef GMW_GLOSSY_DETECT_PIN
#define GLOSSY_DETECT_ON         PIN_SET(GMW_GLOSSY_DETECT_PIN)
#define GLOSSY_DETECT_OFF        PIN_CLR(GMW_GLOSSY_DETECT_PIN)
#else
#define GLOSSY_DETECT_ON
#define GLOSSY_DETECT_OFF
#endif

/*---------------------------------------------------------------------------*/
PROCESS(gmw_noise_detection, "GMW detect noise");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gmw_noise_detection, ev, data)
{

  static uint8_t end_noise_detect;
  static uint8_t i;
  static uint8_t max_loop_counter = 1;

  PROCESS_BEGIN();

  /* main loop of this task */
  while(1) {
    /*
     * the gmw_noise_detection task should not do anything until it is
     * explicitly granted permission (by receiving a poll event) by GMW
     */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

#ifdef PLATFORM_SKY

    /* reset variables */
    counter                 = 0;
    counter_high_noise      = 0;
    end_noise_detect        = 0;

    /* wait a bit before starting the noise detection*/
    rtimer_ext_clock_t now = GMW_RTIMER_NOW();
    while(GMW_RTIMER_NOW() < (now + 10));

    while(!end_noise_detect){

      GLOSSY_DETECT_ON;
      /* check the CCA pin */
      counter++;
      NOISE_DETECT_ON;
      counter_high_noise += !(CC2420_CCA_IS_1); // ~4us on tmote sky
      NOISE_DETECT_OFF;

      /* busy-loop */
      for(i = 0; i<max_loop_counter ; i++) {
        /* check if communication is still ongoing */
        end_noise_detect = !gmw_communication_active();
        if(end_noise_detect) {
          /* if not, flag the end of the noise detection process */
          break;
        }
      }
      GLOSSY_DETECT_OFF;
    }

#elif defined PLATFORM_DPP_CC430

    /* reset variables */
    rssi_value              = 0;
    rssi_value_high_noise   = 0;
    counter                 = 0;
    counter_high_noise      = 0;
    end_noise_detect        = 0;

    while(!end_noise_detect){

      /* sample RSSI sample */
      NOISE_DETECT_ON;
      rssi_value_last = gmw_get_rssi_last(); // ~5us on cc430
      NOISE_DETECT_OFF;

      /* If returned RSSI is 0, the radio function timed out */
      if(rssi_value_last) {

        /* check if RSSI is higher than the defined threshold */
        if(rssi_value_last > GMW_CONF_HIGH_NOISE_THRESHOLD) {
          rssi_value_high_noise += rssi_value_last;
          counter_high_noise++;
        }

        /* store all values */
        rssi_value += rssi_value_last;
        counter++;
      }

      GLOSSY_DETECT_ON;
      /* busy-loop */
      // one iteration of the loop takes ~2.5us in most cases
      // -> sometimes much longer (up to 100us)
      //    because of Glossy-related interrupts
      for(i = 0; i<max_loop_counter ; i++) {
        /* check if communication is still ongoing */
        end_noise_detect = !gmw_communication_active();
        if(end_noise_detect) {
          /* if not, flag the end of the noise detection process */
          break;
        }
      }
      GLOSSY_DETECT_OFF;
    }

#else
  #error "noise detection feature not implemented for the target platform"
#endif

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
gmw_noise_detection_poll(void)
{
  process_poll(&gmw_noise_detection);
}
/*---------------------------------------------------------------------------*/
void
gmw_noise_detection_init(void)
{
  DEBUG_PRINT_MSG_NOW("Starting '%s'", gmw_noise_detection.name);
  process_start(&gmw_noise_detection, NULL);
}
/*---------------------------------------------------------------------------*/
int8_t
gmw_get_rssi_average(void)
{
#ifdef PLATFORM_SKY
  DEBUG_PRINT_WARNING("RSSI measurement not available on Tmote Sky.");
  return 0;
#elif defined PLATFORM_DPP_CC430
  if(counter) {
    return (rssi_value / counter) ;

  } else {
    return -127;
  }
#endif
}
/*---------------------------------------------------------------------------*/
int8_t
gmw_get_rssi_high_noise_average(void)
{
#ifdef PLATFORM_SKY
  DEBUG_PRINT_WARNING("RSSI measurement not available on Tmote Sky.");
  return 0;
#elif defined PLATFORM_DPP_CC430
  return (rssi_value_high_noise / counter_high_noise) ;
#endif
}
/*---------------------------------------------------------------------------*/
uint16_t
gmw_get_noise_measurement_total_samples(void)
{
  return (counter) ;
}
/*---------------------------------------------------------------------------*/
uint16_t
gmw_get_noise_measurement_high_noise_samples(void)
{
  return (counter_high_noise) ;
}
/*---------------------------------------------------------------------------*/
uint8_t
gmw_high_noise_test(void)
{
  return (counter_high_noise >= GMW_CONF_HIGH_NOISE_MIN_COUNT) ;
}
/*---------------------------------------------------------------------------*/
#endif /* GMW_CONF_USE_NOISE_DETECTION */


/** @} */

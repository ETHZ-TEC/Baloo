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
 *            Prototypes for the noise detection process and its API.
 */

#ifndef GMW_NOISE_DETECT_H_
#define GMW_NOISE_DETECT_H_

/**
 * @brief     Poll the noise detection process
 */
void gmw_noise_detection_poll(void);
/*---------------------------------------------------------------------------*/

/**
 * @brief     Initialize the noise detection process
 */
void gmw_noise_detection_init(void);
/*---------------------------------------------------------------------------*/

/**
 * @brief     Get the RSSI measurement during the last flood
 * @return    Average of all RSSI measurements
 */
int8_t gmw_get_rssi_average(void);
/*---------------------------------------------------------------------------*/

/**
 * @brief     Get the high-noise RSSI measurement during the last flood
 * @return    Average of RSSI values being above the configured threshold
 *            (i.e., GMW_CONF_HIGH_NOISE_THRESHOLD)
 */
int8_t gmw_get_rssi_high_noise_average(void);
/*---------------------------------------------------------------------------*/

/**
 * @brief     Get the number of noise measurements during the last flood
 * @return    Total number of samples collected
 */
uint16_t gmw_get_noise_measurement_total_samples(void);
/*---------------------------------------------------------------------------*/

/**
 * @brief     Get the number of high-noise noise measurements during
 *            the last flood
 * @return    Number of samples collected that are above the configured
 *            threshold (i.e., GMW_CONF_HIGH_NOISE_THRESHOLD)
 */
uint16_t gmw_get_noise_measurement_high_noise_samples(void);
/*---------------------------------------------------------------------------*/

/**
 * @brief     Assert if there was 'high-noise' during the last flood
 * @return    True is at least GMW_CONF_HIGH_NOISE_MIN_COUNT RSSI samples
 *            were above the configured threshold GMW_CONF_HIGH_NOISE_THRESHOLD.
 *            False otherwise.
 */
uint8_t gmw_high_noise_test(void);
/*---------------------------------------------------------------------------*/


/** @} */

#endif /* GMW_NOISE_DETECT_H_ */

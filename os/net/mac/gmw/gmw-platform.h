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
 *         Jonas Baechli   jonas.baechli@bluewin.ch
 *         Reto Da Forno   rdaforno@ee.ethz.ch
 *         Romain Jacob    jacobr@ethz.ch
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup  gmw
 * @{
 */
/*---------------------------------------------------------------------------*/

/**
 * \file
 *            Header file for all platform-dependent functions.
 */

#ifndef GMW_PLATFORM_H_
#define GMW_PLATFORM_H_


/**
 * @brief                       GMW-related initialization; everything which
 *                              is not already handled by Contiki.
 *                              Called in gmw_start().
 */
void gmw_platform_init(void);

/**
 * @brief                       Configure the packet length expected by the
 *                              radio. Can be left empty if not supported..
 * @param length                Number of bytes to expect.
 */
void gmw_set_maximum_packet_length(uint8_t length);

/**
 * @brief                       Configure the radio channel to be used.
 * @param channel               ID of the radio channel to be used.
 */
void gmw_set_rf_channel(gmw_rf_tx_channel_t channel);

/**
 * @brief                       Configure the TX power to be used
 * @param   power               TX power in dBm, will be set to the closest
 *                              value available (equal or larger).
 */
void gmw_set_tx_power(gmw_rf_tx_power_t power);

/**
 * @brief                       Implementation of the noise detection feature.
 * @returns                     True if high-noise was detected during the last
 *                              slot, false otherwise.
 *                              Returns false if the noise detection feature
 *                              is disabled.
 */
uint8_t gmw_high_noise_detected();

/**
 * @brief                       Check is a synchronous transmission primitive
 *                              is currently running.
 * @returns                     True or false.
 *
 * @note                        Currently does not support multiple primitives
 *                              (i.e., valid for Glossy only).
 */
//TODO: extend to multiple primitives
uint8_t gmw_communication_active();

/**
 * @brief                       Get the last available RSSI measurement from
 *                              the radio module.
 * @returns                     RSSI value in dBm.
 */
int8_t gmw_get_rssi_last();

/** @} */

#endif /* GMW_PLATFORM_H_ */

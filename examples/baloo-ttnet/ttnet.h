/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich).
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
 *         Romain Jacob    jacobr@ethz.ch
 */
/**
 *
 * \file
 *         TTW-specific types and helpers
 */

/*---------------------------------------------------------------------------*/
/*- CUSTOM TYPES ------------------------------------------------------------*/

typedef struct __attribute__((packed)) ttw_mode {
  uint32_t hyperperiod;       // Expressed in GMW_CONF_PERIOD_TIME_BASE
  uint16_t first_round_id;    /* The index of the first round mapped to
                               * this mode in the ttw_round array
                               */
} ttw_mode_t;

//TODO: consider the utility of slot_alloc...
typedef struct __attribute__((packed)) slot_alloc {
  uint16_t slot_id;           // An index of allocated slot
  uint16_t message_id;        // The corresponding message mapped to this slot
} slot_alloc_t;

typedef struct __attribute__((packed)) ttw_round {
  uint32_t start_time_offset; // Offset from the mode hyperperiod, expressed in GMW_CONF_PERIOD_TIME_BASE
  uint16_t n_slots;           // Number of slots in the round
  uint8_t mode_id;            // The index of the corresponding mode in the ttw_mode_array
} ttw_round_t;
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*- HELPERS -----------------------------------------------------------------*/

/* Beacon is 2-byte long:
 * The first byte contains the round ID.
 * The second byte contains:
 * - The switching bit SB (most significant bit)
 * - The mode ID (7 least significant bits)
 */

/* Round */
#define TTW_SET_BEACON_ROUND(c,roundID) \
  ((c)->user_bytes[0] = (uint8_t)(roundID))
#define TTW_GET_BEACON_ROUND(c)   \
  ((c)->user_bytes[0])

/* Mode */
#define TTW_SET_BEACON_MODE(c,modeID) \
  ((c)->user_bytes[1] = (((0x7f) & (uint8_t)(modeID))) | (((c)->user_bytes[1]) & (0x80)))
#define TTW_GET_BEACON_MODE(c) \
  (((c)->user_bytes[1]) & (0x7f))

/* Switching bit */
#define TTW_SET_BEACON_SB(c) \
  ((c)->user_bytes[1] |= (0x80))
#define TTW_GET_BEACON_SB(c) \
  ((((c)->user_bytes[1]) & (0x80)) >> 7)
#define TTW_CLR_BEACON_SB(c) \
  ((c)->user_bytes[1] &= (0x7f))

/**
 * @brief                       TTW node role identifiers
 */
typedef enum {
  TTW_SENDER = 0,
  TTW_RECEIVER,
  TTW_FORWARDER
} ttw_role_t;
/*---------------------------------------------------------------------------*/

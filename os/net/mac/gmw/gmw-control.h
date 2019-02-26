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
 * @addtogroup  gmw
 * @{
 */
/*---------------------------------------------------------------------------*/

/**
 * \file
 *            This file contains helpers and function prototypes to handle
 *            the information contained in the control structure.
 */

#ifndef GMW_CONTROL_H_
#define GMW_CONTROL_H_

#define GMW_CONTROL_SCHED_N_SLOTS_MASK              (0x1fff)
#define GMW_CONTROL_SCHED_N_SLOTS_CONFIG_MASK       (0x8000)
#define GMW_CONTROL_SCHED_N_SLOTS_SLOT_CONFIG_MASK  (0x4000)
#define GMW_CONTROL_SCHED_N_SLOTS_USER_BYTES_MASK   (0x2000)

/**
 * @brief                       Return the number of data slots
 *                              from schedule section
 */
#define GMW_SCHED_N_SLOTS(s)    ((s)->n_slots & GMW_CONTROL_SCHED_N_SLOTS_MASK)

/**
 * @brief                       Slot assignment identifying a contention slot.
 */
#define GMW_SLOT_CONTENTION     0xffff  /* a contention slot */

/**
 * @brief                       Helpers to set, clear, or check if the control
 *                              packet contains a config section.
 */
#define GMW_CONTROL_HAS_CONFIG(c)  (((c)->schedule.n_slots & \
                                  GMW_CONTROL_SCHED_N_SLOTS_CONFIG_MASK) > 0)
#define GMW_CONTROL_SET_CONFIG(c)  ((c)->schedule.n_slots |= \
                                  GMW_CONTROL_SCHED_N_SLOTS_CONFIG_MASK)
#define GMW_CONTROL_CLR_CONFIG(c)  ((c)->schedule.n_slots &= \
                                  ~GMW_CONTROL_SCHED_N_SLOTS_CONFIG_MASK)

/**
 * @brief                       Helpers to set, clear, or check if the control
 *                              packet contains a slot config section.
 */
#define GMW_CONTROL_HAS_SLOT_CONFIG(c)  (((c)->schedule.n_slots & \
                            GMW_CONTROL_SCHED_N_SLOTS_SLOT_CONFIG_MASK) > 0)
#define GMW_CONTROL_SET_SLOT_CONFIG(c)  ((c)->schedule.n_slots |= \
                            GMW_CONTROL_SCHED_N_SLOTS_SLOT_CONFIG_MASK)
#define GMW_CONTROL_CLR_SLOT_CONFIG(c)  ((c)->schedule.n_slots &= \
                            ~GMW_CONTROL_SCHED_N_SLOTS_SLOT_CONFIG_MASK)

/**
 * @brief                       Helpers to set, clear, or check if the control
 *                              packet contains user bytes.
 */
#define GMW_CONTROL_HAS_USER_BYTES(c)  (((c)->schedule.n_slots & \
                            GMW_CONTROL_SCHED_N_SLOTS_USER_BYTES_MASK) > 0)
#define GMW_CONTROL_SET_USER_BYTES(c)  ((c)->schedule.n_slots |= \
                            GMW_CONTROL_SCHED_N_SLOTS_USER_BYTES_MASK)
#define GMW_CONTROL_CLR_USER_BYTES(c)  ((c)->schedule.n_slots &= \
                            ~GMW_CONTROL_SCHED_N_SLOTS_USER_BYTES_MASK)

/**
 * @brief                       Initializes the control structure on the
 *                              application side with all default parameters.
 *
 * @param control               Pointer to the control structure.
 */
void gmw_control_init(gmw_control_t* control);

/**
 * @brief                       Rearrange the control struct into a byte buffer
 *                              to save radio transmission time
 * @param control               Pointer to the control struct to send
 * @param buffer                Pointer to the send buffer
 * @param len                   Size of the send buffer
 * @return                      The used size of the buffer (in Bytes),
 *                              or 0 if it fails (e.g., buffer too small)
 *
 * @note                        The payload size (i.e., sent control
 *                              information) must be at least one byte. Glossy
 *                              does not execute in case of 0-byte payload.
 */
uint8_t gmw_control_compile_to_buffer(const gmw_control_t* control,
                                      uint8_t* buffer, 
                                      uint8_t len);

/**
 * @brief                       Reconstruct the original control struct from
 *                              the received buffer
 * @param control               Pointer to the control struct to be filled
 * @param buffer                Pointer to the received buffer
 * @param len                   of the valid data in buffer
 * @return                      1 if successful, 0 otherwise
 */
uint8_t gmw_control_decompile_from_buffer(gmw_control_t* control,
                                          uint8_t* buffer, 
                                          uint8_t len);

/** @} */

#endif /* GMW_CONTROL_H_ */

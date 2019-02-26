/*
 * Copyright (c) 2018-2019, Swiss Federal Institute of Technology (ETH Zurich).
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


#ifndef __SB_H__
#define __SB_H__

/*---------------------------------------------------------------------------*/
/*
 * Helpers
 */

#define STROBE_BIT_MASK                 0x02
#define SYNC_BIT_MASK                   0x01
#define IS_POTENTIAL_PARENT(slot)       (uint8_t)((potential_parent_bitmap & ( (uint64_t)1 << (slot - 2*current_rr_pairs) )) >> (slot - 2*current_rr_pairs))
#define IS_ACTIVE_SLOT(slot)            (uint8_t)((active_node_bitmap & ( (uint64_t)1 << (slot) )) >> (slot))
#define NOT_IN_LIST                     0xff
#define NOT_ASSIGNED                    0xff
#define MYSELF                          NOT_IN_LIST - 1

/*---------------------------------------------------------------------------*/
/*
 * Evaluation test definition
 */
#define SOURCES_LIST                    { 2,3,15,18,23,28,4,8,20,22,25,26 }
#define NUM_SOURCES                       12
#define NUM_SOURCES_CASE                  6


/*---------------------------------------------------------------------------*/
/*
 * Data type definitions
 */

typedef uint16_t etx_value_t;

/*---------------------------------------------------------------------------*/
/*
 * Packet definitions
 */

typedef struct {
    uint16_t        req_src;
}
__attribute__((packed))
sb_req_struct;

typedef struct {
    uint16_t        ack_src;
    uint8_t         assigned_slot;
}
__attribute__((packed))
sb_ack_struct;

typedef struct {
    etx_value_t     etx_value;
}
__attribute__((packed))
sb_strobe_struct;

typedef struct {
    uint8_t         payload[SB_PAYLOAD_LENGTH];
    uint8_t         parents[SB_NUMBER_PARENTS];
}
__attribute__((packed))
sb_data_struct;

/*---------------------------------------------------------------------------*/
/*
 * Topology Information structures
 */

typedef struct {
    etx_value_t     etx_value;
    uint8_t         assigned_slot;
}__attribute__((packed))
sb_parent_info_struct;

typedef struct {
    etx_value_t             min_etx;                        // current best  ETX
    etx_value_t             max_etx;                        // current worst ETX (of the reported ones)
    sb_parent_info_struct   parent_info[SB_NUMBER_PARENTS]; // info of the reported parents

}__attribute__((packed))
sb_topology_struct;


#endif /* __SB_H__ */

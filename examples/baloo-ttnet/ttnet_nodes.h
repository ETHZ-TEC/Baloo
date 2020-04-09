/*
 * Copyright (c) 2020, Swiss Federal Institute of Technology (ETH Zurich).
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
 *         List of node IDs and custom identifiers
 */

#ifndef __TTNET_NODES_H__
#define __TTNET_NODES_H__

#ifdef FLOCKLAB
	static const uint16_t static_nodes[] = {2,4,5,6,7,8,9,10,11};
#else
	static const uint16_t static_nodes[] = { 7, 1, 1, 2 };
#endif /* FLOCKLAB */

/* Custom node identifiers */
#define NODE0 static_nodes[0]
#define NODE1 static_nodes[1]
#define NODE2 static_nodes[2]
#define NODE4 static_nodes[3]

#define SENSOR1 	static_nodes[0]
#define SENSOR2 	static_nodes[1]
#define ACTUATOR	static_nodes[2]
#define CONTROLLER	static_nodes[3]

#endif /*__TTNET_NODES_H__*/

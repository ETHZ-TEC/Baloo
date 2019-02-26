/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

/*
 * contiki configuration, architecture specific
 */

/* include the project config */
#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */
/*---------------------------------------------------------------------------*/
#include "dpp-def.h"
/*---------------------------------------------------------------------------*/

/* Compiler configurations */
#define CLIF
#define CCIF

#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON        0
#endif /* ENERGEST_CONF_ON */

#ifndef AUTOSTART_ENABLE
#define AUTOSTART_ENABLE        1
#endif /* AUTOSTART_ENABLE */

#ifndef NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_WITH_IPV6 0
#endif /* NETSTACK_CONF_WITH_IPV6 */

#ifndef CLOCK_CONF_SECOND
#define CLOCK_CONF_SECOND       32
#endif /* CLOCK_CONF_SECOND */

#ifndef STACK_CHECK_CONF_ENABLED
#define STACK_CHECK_CONF_ENABLED  0
#endif /* STACK_CHECK_CONF_ENABLED */


#endif /* __CONTIKI_CONF_H__ */

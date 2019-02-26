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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author:  Reto Da Forno
 */

#include <string.h>
#include "contiki.h"
#include "debug-print.h"
#include "gpio.h"
#include "node-id.h"
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE    "TestApp"
#define LOG_LEVEL LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "LPMTest task");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  static struct  etimer timer;

  PROCESS_BEGIN();

  /* Setup a periodic timer that expires after 5 seconds. */
  etimer_set(&timer, CLOCK_SECOND * 5);

  /* main loop */
  while(1) {
    LED_TOGGLE(LED_STATUS);
    LOG_INFO("hello world!\n");
    /* Wait for the periodic timer to expire and then restart the timer. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    etimer_reset(&timer);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

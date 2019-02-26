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
#include "gpio.h"
#include "node-id.h"
#include "basic-radio.h"
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE    "TestApp"
#define LOG_LEVEL LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/
static const char* message = "hello world!";
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "HelloWorld task");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  static struct  etimer timer;
  static uint8_t rx_buffer[RF_CONF_MAX_PKT_LEN + 1];
  static uint8_t rcvd_bytes;

  PROCESS_BEGIN();

  /* configure GPIO as outputs */
  PIN_CFG_OUT(RADIO_START_PIN);
  PIN_CFG_OUT(RADIO_RX_PIN);
  PIN_CFG_OUT(RADIO_TX_PIN);
  PIN_CFG_OUT(LED_STATUS);

  /* Setup a periodic timer that expires after 2 seconds. */
  etimer_set(&timer, CLOCK_SECOND * 2);

  /* main loop */
  while(1) {

    if(node_id == SENDER_ID) {

      LED_ON(LED_STATUS);
      if(radio_send((uint8_t*)message, strlen(message), 0)) {
        LOG_INFO("packet sent (%u bytes)\n", strlen(message));
      } else {
        LOG_INFO("send failed\n");
      }
      LED_OFF(LED_STATUS);

    } else {

      do {
        LOG_INFO("listening...\n");
        etimer_restart(&timer);
        LED_ON(LED_STATUS);
        rcvd_bytes = radio_rcv(rx_buffer, 1000);  /* 1000ms timeout */
        LED_OFF(LED_STATUS);
        if(rcvd_bytes) {
          /* print the received payload */
          rx_buffer[rcvd_bytes] = 0;
          LOG_INFO("%u bytes received: %s\n", rcvd_bytes, (char*)rx_buffer);
        }
      } while(!rcvd_bytes);
    }

    /* Wait for the periodic timer to expire and then restart the timer. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    etimer_reset(&timer);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

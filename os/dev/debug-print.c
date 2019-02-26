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
 * Author:  Reto Da Forno
 */

#include "debug-print.h"
#include "node-id.h"
#include "gpio.h"
#include "sys/log.h"
#define LOG_MODULE "DebugPrint"
#define LOG_LEVEL LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/
#ifdef DEBUG_PRINT_CONF_TASK_ACT_PIN
#define DEBUG_PRINT_TASK_ACTIVE       PIN_CLR(DEBUG_PRINT_CONF_TASK_ACT_PIN);\
                                      PIN_SET(DEBUG_PRINT_CONF_TASK_ACT_PIN)
#define DEBUG_PRINT_TASK_SUSPENDED    PIN_CLR(DEBUG_PRINT_CONF_TASK_ACT_PIN)
#else
#define DEBUG_PRINT_TASK_ACTIVE
#define DEBUG_PRINT_TASK_SUSPENDED
#endif
/*---------------------------------------------------------------------------*/
/* helper macros */
#if DEBUG_PRINT_CONF_DISABLE_UART
  #define DEBUG_PRINT_UART_ENABLE     DEBUG_PRINT_CONF_UART_ENABLE()
  #define DEBUG_PRINT_UART_DISABLE    DEBUG_PRINT_CONF_UART_DISABLE()
#else /* DEBUG_PRINT_CONF_DISABLE_UART */
  #define DEBUG_PRINT_UART_ENABLE
  #define DEBUG_PRINT_UART_DISABLE
#endif /* DEBUG_PRINT_CONF_DISABLE_UART */
/*---------------------------------------------------------------------------*/
struct printbuf {
  uint8_t *data;
  uint16_t size;
  uint16_t put_idx, get_idx;
  uint16_t cnt;
};
void printbuf_put(const char* str);
void printbuf_flush(void);
/*---------------------------------------------------------------------------*/
const char* debug_print_lvl_to_string[NUM_OF_DEBUG_PRINT_LEVELS] = { \
  "CRIT: ", "ERROR:", "WARN: ", "INFO: ", "DBG:  " };
/* global buffer, required to compose the messages */
char debug_print_buffer[DEBUG_PRINT_CONF_MSG_LEN]; 
static struct  printbuf dbg_printbuf = { 0 };
static uint8_t dbg_printbuf_data[DEBUG_PRINT_CONF_BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
PROCESS(debug_print_process, "Debug Print Task");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(debug_print_process, ev, data)
{
  PROCESS_BEGIN();

  dbg_printbuf.data = dbg_printbuf_data;
  dbg_printbuf.size = DEBUG_PRINT_CONF_BUFFER_SIZE;

#ifdef DEBUG_PRINT_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(DEBUG_PRINT_CONF_TASK_ACT_PIN);
#endif

#if DEBUG_PRINT_CONF_STACK_GUARD
  /* fill all unused stack memory with a dummy value */
  uint16_t* addr;
  uint16_t* end = (uint16_t*)&addr - 1;  /* stop at the current stack height */
  for(addr = (uint16_t*)(DEBUG_PRINT_CONF_STACK_GUARD); addr < end; addr++) {
    *addr = 0xaaaa;
  }
  LOG_INFO("Buffer size: %uB, max stack size: %uB" DEBUG_PRINT_CONF_EOL,
           DEBUG_PRINT_CONF_BUFFER_SIZE,
           (SRAM_START + SRAM_SIZE - DEBUG_PRINT_CONF_STACK_GUARD -8));
#else
  LOG_INFO("Buffer size: %uB" DEBUG_PRINT_CONF_EOL,
           DEBUG_PRINT_CONF_BUFFER_SIZE);
#endif /* DEBUG_PRINT_CONF_STACK_GUARD */

  while(1) {
    /* suspend this task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /* wait until we get polled by another thread */
    DEBUG_PRINT_TASK_ACTIVE;

    DEBUG_PRINT_UART_ENABLE;
    printbuf_flush();
    DEBUG_PRINT_UART_DISABLE;

#if DEBUG_PRINT_CONF_STACK_GUARD
    /* check if the stack might be corrupt (check 8 bytes) */
    if(*(uint16_t*)DEBUG_PRINT_CONF_STACK_GUARD != 0xaaaa       || 
       *(uint16_t*)(DEBUG_PRINT_CONF_STACK_GUARD + 2) != 0xaaaa || 
       *(uint16_t*)(DEBUG_PRINT_CONF_STACK_GUARD + 4) != 0xaaaa || 
       *(uint16_t*)(DEBUG_PRINT_CONF_STACK_GUARD + 6) != 0xaaaa) {
      DEBUG_PRINT_FATAL("FATAL ERROR: Stack overflow detected");
    }
#endif /* DEBUG_PRINT_CONF_STACK_GUARD */

    DEBUG_PRINT_TASK_SUSPENDED;
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
debug_print_init(void)
{
  LOG_INFO("Starting '%s'" DEBUG_PRINT_CONF_EOL, debug_print_process.name);
  process_start(&debug_print_process, NULL);
}
/*---------------------------------------------------------------------------*/
void
debug_print_poll(void)
{
  process_poll(&debug_print_process);
}
/*---------------------------------------------------------------------------*/
void
#if DEBUG_PRINT_CONF_PRINT_FILENAME
debug_print_msg(unsigned long timestamp,
                debug_level_t level,
                char *data,
                char *filename)
#else /* DEBUG_PRINT_CONF_PRINT_FILENAME */
debug_print_msg(unsigned long timestamp,
                debug_level_t level,
                char *data)
#endif /* DEBUG_PRINT_CONF_PRINT_FILENAME */
{
  char tmp[32];
  printbuf_put("[");
#if DEBUG_PRINT_CONF_PRINT_DBGLEVEL
  printbuf_put(debug_print_lvl_to_string[level]);
#endif /* DEBUG_PRINT_CONF_PRINT_DBGLEVEL */
#if DEBUG_PRINT_CONF_PRINT_NODEID
  snprintf(tmp, 32, DEBUG_PRINT_CONF_PRINT_DBGLEVEL ? " %u " : "%u ", node_id);
  printbuf_put(tmp);
#endif /* DEBUG_PRINT_CONF_PRINT_NODEID */
#if DEBUG_PRINT_CONF_PRINT_FILENAME
  snprintf(tmp, 32,
           (DEBUG_PRINT_CONF_PRINT_DBGLEVEL ||
            DEBUG_PRINT_CONF_PRINT_NODEID) ? " %-10s" : "%-10s",
           filename);
  printbuf_put(tmp);
#endif /* DEBUG_PRINT_CONF_PRINT_FILENAME */
#if DEBUG_PRINT_CONF_PRINT_TIMESTAMP
  snprintf(tmp, 32, 
           (DEBUG_PRINT_CONF_PRINT_DBGLEVEL ||
            DEBUG_PRINT_CONF_PRINT_NODEID   ||
            DEBUG_PRINT_CONF_PRINT_FILENAME) ?  " %4lu" : "%4lu",
           timestamp);
  printbuf_put(tmp);
#endif /* DEBUG_PRINT_CONF_PRINT_TIMESTAMP */
  printbuf_put("] ");
  printbuf_put(data);
  printbuf_put(DEBUG_PRINT_CONF_EOL);
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg_now(char *data)
{
  if(data) {
    DEBUG_PRINT_UART_ENABLE;
    printf(data);
    printf(DEBUG_PRINT_CONF_EOL);
    DEBUG_PRINT_UART_DISABLE;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
debug_print_get_max_stack_size(void)
{
#if DEBUG_PRINT_CONF_STACK_GUARD
  uint16_t* addr = (uint16_t*)DEBUG_PRINT_CONF_STACK_GUARD;
  uint16_t* end = (uint16_t*)(SRAM_START + SRAM_SIZE);
  for(; addr < end; addr++) {
    if(*addr != 0xaaaa) {
      return ((uint16_t)end - (uint16_t)addr);
    }
  }
#endif /* DEBUG_PRINT_CONF_STACK_GUARD */
  return 0;
}
/*---------------------------------------------------------------------------*/
void
printbuf_put(const char* str)
{
  static const char line_cut[] = "~" DEBUG_PRINT_CONF_EOL;

  if(dbg_printbuf.cnt == dbg_printbuf.size)
    return;
  uint16_t lim = dbg_printbuf.size - strlen(line_cut);
  while(*str && dbg_printbuf.cnt < lim) {
    dbg_printbuf.data[dbg_printbuf.put_idx++] = *str++;
    if(dbg_printbuf.put_idx == dbg_printbuf.size) {
      dbg_printbuf.put_idx = 0;
    }
    dbg_printbuf.cnt++;
  }
  if(*str) {
    /* no more space -> add special character + newline */
    str = line_cut;
    while(*str) {
      dbg_printbuf.data[dbg_printbuf.put_idx++] = *str++;
      if(dbg_printbuf.put_idx == dbg_printbuf.size) {
        dbg_printbuf.put_idx = 0;
      }
    }
    dbg_printbuf.cnt += strlen(line_cut);
  }
}
/*---------------------------------------------------------------------------*/
void
printbuf_flush(void)
{
  while(dbg_printbuf.cnt)
  {
    uint8_t c = dbg_printbuf.data[dbg_printbuf.get_idx++];
    if(dbg_printbuf.get_idx == dbg_printbuf.size) {
      dbg_printbuf.get_idx = 0;
    }
    putchar(c);
    dbg_printbuf.cnt--;
  }
}
/*---------------------------------------------------------------------------*/

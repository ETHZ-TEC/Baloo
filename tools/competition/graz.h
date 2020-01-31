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
 * Author:  Romain Jacob
 */

#ifndef GRAZ_H_
#define GRAZ_H_

/* platform dependent GPIO definitions */
#ifdef PLATFORM_SKY

  /* the following pins assignments are given by the Graz testbed */
  #define GPIO17  ADC0  // 0x80
  #define GPIO4   ADC1  // 0x40
  #define GPIO18  ADC2  // 0x20
  #define GPIO27  GPIO0 // 0x10
  #define GPIO22  ADC7  // 0x08
  #define GPIO23  GPIO2 // 0x04
  #define GPIO24  GPIO3 // 0x02
  #define GPIO25  ADC6  // 0x01

#endif /* PLATFORM_... */


#ifdef GPIO17
  #define GPIO17_INIT  {  PIN_UNSEL(GPIO17); \
                          PIN_CFG_OUT(GPIO17); \
                          PIN_CLR(GPIO17); }
#else  /* GPIO17 */
  #define GPIO17_INIT
#endif /* GPIO17 */

#ifdef GPIO4
  #define GPIO4_INIT  {   PIN_UNSEL(GPIO4); \
                          PIN_CFG_OUT(GPIO4); \
                          PIN_CLR(GPIO4); }
#else  /* GPIO4 */
  #define GPIO4_INIT
#endif /* GPIO4 */

#ifdef GPIO18
  #define GPIO18_INIT  {  PIN_UNSEL(GPIO18); \
                          PIN_CFG_OUT(GPIO18); \
                          PIN_CLR(GPIO18); }
#else  /* GPIO18 */
  #define GPIO18_INIT
#endif /* GPIO18 */

#ifdef GPIO27
  #define GPIO27_INIT  {  PIN_UNSEL(GPIO27); \
                          PIN_CFG_OUT(GPIO27); \
                          PIN_CLR(GPIO27); }
#else  /* GPIO27 */
  #define GPIO27_INIT
#endif /* GPIO27 */

#ifdef GPIO22
  #define GPIO22_INIT  {  PIN_UNSEL(GPIO22); \
                          PIN_CFG_OUT(GPIO22); \
                          PIN_CLR(GPIO22); }
#else  /* GPIO22 */
  #define GPIO22_INIT
#endif /* GPIO22 */

#ifdef GPIO23
  #define GPIO23_INIT  {  PIN_UNSEL(GPIO23); \
                          PIN_CFG_OUT(GPIO23); \
                          PIN_CLR(GPIO23); }
#else  /* GPIO23 */
  #define GPIO23_INIT
#endif /* GPIO23 */

#ifdef GPIO24
  #define GPIO24_INIT  {  PIN_UNSEL(GPIO24); \
                          PIN_CFG_OUT(GPIO24); \
                          PIN_CLR(GPIO24); }
#else  /* GPIO24 */
  #define GPIO24_INIT
#endif /* GPIO24 */

#ifdef GPIO25
  #define GPIO25_INIT  {  PIN_UNSEL(GPIO25); \
                          PIN_CFG_OUT(GPIO25); \
                          PIN_CLR(GPIO25); }
#else  /* GPIO25 */
  #define GPIO25_INIT
#endif /* GPIO25 */


#ifdef GRAZ
  #define GRAZ_INIT()     { \
                                GPIO17_INIT; \
                                GPIO4_INIT; \
                                GPIO18_INIT; \
                                GPIO27_INIT; \
                                GPIO22_INIT; \
                                GPIO23_INIT; \
                                GPIO24_INIT; \
                                GPIO25_INIT; \
                              }
#endif /* GRAZ */


#endif /* GRAZ_H_ */

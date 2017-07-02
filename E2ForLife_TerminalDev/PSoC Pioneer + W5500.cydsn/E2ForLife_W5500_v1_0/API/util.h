/**
 * \addtogroup e2forlife_w5500
 * @{
 */
/**
 * Copyright (c) 2015, E2ForLife.com
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of E2ForLife.com nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file util.c
 * \author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Changes: Carlos Diaz (carlos.santiago.diaz@gmail.com) 2017
 * Header file for the util functions.
 */

#ifndef UTIL_H
#define UTIL_H

#include <cyfitter.h>
#include <cytypes.h> // stdint.h

uint32_t ParseIP( const char* ipString );
cystatus ParseMAC(const char *macString, uint8_t *mac);
void StringMAC(uint8_t *mac, char *macString);
void StringIP( uint32_t ip, char *ipString );
int Base64Encode(const void* data_buf, int dataLength, char* result, int resultSize);
int Base64Decode (char *in, int inLen, uint8_t *out, int *outLen);
uint32_t IPADDRESS( const uint8_t x1, const uint8_t x2, const uint8_t x3, const uint8_t x4 );

#endif // UTIL_H

/* [] END OF FILE */

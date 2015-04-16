#if !defined( ETH_UTIL_H)
	#define ETH_UTIL_H
/* ------------------------------------------------------------------------ */
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
 * \file iot.h
 * \author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Header for the IoT Utility functions, defines macros and function
 * prototypes for use of the IoT helpers.
 */
/* ======================================================================== */
	
#include <cytypes.h>
	
#define IOT_IPADDRESS(x1,x2,x3,x4)   ( (uint32)(x1&0x000000FF) + (uint32)((x2<<8)&0x0000FF00) + (uint32)((x3<<16)&0x00FF0000) + ((uint32)(x4<<24)&0xFF000000 ))

uint32 IOT_ParseIP( const char* ipString );
cystatus IOT_ParseMAC(const char *macString, uint8 *mac);
void IOT_StringMAC(uint8 *mac, char *macString);
void IOT_StringIP( uint32 ip, char *ipString );
int IOT_base64encode(const void* data_buf, int dataLength, char* result, int resultSize);
int IOT_base64decode (char *in, int inLen, uint8 *out, int *outLen);


#endif
/** @} */
/* [] END OF FILE */

/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#if !defined( ETH_UTIL_H)
	#define ETH_UTIL_H
	
#include <cytypes.h>
	
#define IOT_IPADDRESS(x1,x2,x3,x4)   ( (uint32)(x1&0x000000FF) + (uint32)((x2<<8)&0x0000FF00) + (uint32)((x3<<16)&0x00FF0000) + ((uint32)(x4<<24)&0xFF000000 ))

uint32 IOT_ParseIP( const char* ipString );
cystatus IOT_ParseMAC(const char *macString, uint8 *mac);
void IOT_StringMAC(uint8 *mac, char *macString);
void IOT_StringIP( uint32 ip, char *ipString );
int IOT_base64encode(const void* data_buf, int dataLength, char* result, int resultSize);
int IOT_base64decode (char *in, int inLen, uint8 *out, int *outLen);


#endif
/* [] END OF FILE */

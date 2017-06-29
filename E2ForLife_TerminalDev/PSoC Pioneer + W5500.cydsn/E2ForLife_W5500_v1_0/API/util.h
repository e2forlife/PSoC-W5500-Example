#ifndef UTIL_H
#define UTIL_H

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

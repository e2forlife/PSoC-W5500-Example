/**
 * @addtogroup e2forlife_w5500
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
 * @file util.c
 * @author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Function implementations for IoT utility functions used by the W5500 driver
 */

#include "`$INSTANCE_NAME`.h"
#include "`$INSTANCE_NAME`_util.h"

#include <cytypes.h> // stdint.h, cystatus
#include <CyLib.h> // string.h, ctype.h

#define _HEX2BIN(x) \
( ((x>='0')&&(x<='9'))? (x-'0') : \
  ((x>='a')&&(x<='f'))? ((x-'a')+10) : \
  ((x>='A')&&(x<='F'))? ((x-'A')+10) : 0 )

#define _BIN2HEX(x) \
( (x>9)? ((x-10)+'A') : (x + '0'))

#define _DECODE64_WHITESPACE 64
#define _DECODE64_EQUALS     65
#define _DECODE64_INVALID    66

/**
 * @brief Convert an ASCII string to an IP address.
 *
 * This function parses the XXX.YYY.ZZZ.AAA IPv4 address in to a register
 * format IP address for use with the MAC hardware. The adress must be
 * stored as ASCII decimal with period delimiters between the valeus.  Upon
 * sucessful completion a register image of the IP address is returned,
 * otherwise, 0xFFFFFFFF is returned.
 *
 * @param ipString (char*) buffer containing the IP address to convert.
 * @returns uint32 the converted IP address from teh ASCII-Z string
 * @returns 0xFFFFFFFF Illegal or invalid IP address. Bad Conversion
 */
uint32_t `$INSTANCE_NAME`_ParseIP( const char* ipString )
{
    // Parse a human readable string in to a IP address usable by the hardare
    char digit[5] = {0};
    uint8_t ip[4] = {0};
    uint8_t index = 0;
    uint8_t counter = 0;
    uint8_t dindex = 0;
	
    while ( ( counter < 4 ) && ( (unsigned int)index < strlen( ipString ) ) ) {
        if ( ( '0' <= ipString[index] ) && ( '9' >= ipString[index] ) ) {
            if ( dindex > 3 ) {
                return( 0xFFFFFFFF );
            }
                digit[dindex++] = ipString[index];
        }
		
        if ( ( '.' == ipString[index] ) || ( 0 == ipString[index + 1 ] ) ) {
                digit[dindex] = 0;
                dindex = 0;
                // convert the value and store in the buffer
                ip[counter] = 0;

                while ( 0 != digit[dindex] ) {
                        ip[counter] = ( ip[counter] * 10 ) + ( digit[dindex] - '0' );
                        ++dindex;
                        // ip[counter] = ...
                }
                // reset the digit index to start accumulating digits again
                dindex = 0;
                // move to the next address byte
                ++counter;
        }
        ++index;
    }

    if ( 4 != counter ) {
        return 0xFFFFFFFF;
    } else {
        return `$INSTANCE_NAME`_IPADDRESS( ip[0], ip[1], ip[2], ip[3] );
    }
}

/**
 * @brief Parse a MAC Address string in to a 6-byte mac address
 *
 * A mac address is specified as a string of 6 hex bytes with
 * colon (':') seperating the bytes. An invalidly formed
 * address will only process the values up the error and return CYRET_BAD_DATA
 * otherwise, CYRET_SUCCESS is returned.
 *
 * @param *macString Pointer to the ASCII-Z String containing the MAC address
 * @param *mac Pointer to the 6-byte array to hold the output mac addres
 *
 * @return
 */
cystatus `$INSTANCE_NAME`_ParseMAC( const char* macString, uint8_t* mac )
{

	uint16 index = 0;
	cystatus result = CYRET_SUCCESS;

	for( uint8_t digit = 0;
        ( digit < 6) && ( CYRET_SUCCESS == result ) && ( 0 != macString[index] );
        ++digit ) {

		// process the first nibble
        if ( isxdigit( (int)macString[index] ) ) {
            mac[digit] = _HEX2BIN( macString[index] );
			++index;
			mac[digit] <<= 4;

            if ( isxdigit( (int)macString[index] ) ) {
                mac[digit] += _HEX2BIN( macString[index] );
				++index;
				// now for digits other than digit 5 (the last one) look for
				// the dash seperator.  If there is no dash, return bad data
                if ( digit < 5 ) {
                    if ( ':' != macString[index] ) {
						result = CYRET_BAD_DATA;
					}
					++index; // move conversion pointer to the next value
				}
			} else {
				result = CYRET_BAD_DATA;
			}
		} else {
			result = CYRET_BAD_DATA;
		}
	}
    
	return result;
}

/**
 * @brief Convert a device MAC address (HArdware Address) to ASCII
 *
 * This function takes the passed array of 6 bytes and converts it to an
 * ASCI-Z string containing HEX values and a colon (:) delimiter between bytes
 * of the harware address.  This is a nice fucntion for displaying a MAC
 * address, but otherwise not really required for functionality
 *
 * @param mac (*uint8) array of bytes holding the MAC address
 * @param macString (*char) pointer to an ASCII-Z string buffer to hold output
 *
 * @return
 */
void `$INSTANCE_NAME`_StringMAC( uint8_t* mac, char* macString )
{
	uint8_t index = 0;
	
	// first read the MAC address from the chip
	// and inintialize some locals so that the
	// string formater will function properly
	for( uint8_t digit = 0; digit < 6; ++digit ) {
		// convert the first nibble
        macString[index++] = _BIN2HEX( ( ( mac[digit] >> 4 ) & 0x0F ) );
        macString[index++] = _BIN2HEX( ( mac[digit] & 0x0F ) );
		if ( digit < 5 ) {
			macString[index++] = ':';
		}
		else {
			macString[index] = 0;
		}
	}
}

/**
 * @brief convert an IP address to a ASCII String for printing
 *
 * This function takes the 32-bit IP address from the chip registers and
 * converts it to human-readable ASCII suitable for printing or storing in a
 * log file. This is basically just a useful helper function to prevent having
 * to slog through the hex to determine the IP address ehn debugging.
 *
 * @param ip (uinit32) Binary form IP Address
 * @param ipString (*char) pointer to character buffer to hold the IP address
 *
 * @return
 */
void `$INSTANCE_NAME`_StringIP( uint32_t ip, char* ipString )
{
	uint8_t *ipBytes = (uint8_t*) &ip;
	uint8_t index = 0;
	//int digit = 0;
	uint32_t work = 0;
    uint32_t temp = 0;
	
	//ipBytes = (uint8*)&ip;
	//index = 0;
	for( uint8_t digit = 0; digit < 4; ++digit ) {
		work = ipBytes[digit];
        
		if ( work >= 100 ) {
			temp = work / 100;
			work -= (temp * 100);
			ipString[index++] = '0' + temp;
		}
		if ( work >= 10 ) {
			temp = work / 10;
			work -= (temp * 10);
			ipString[index++] = '0' + temp;
		}
		ipString[index++] = '0' + work;
		if ( digit < 3 ) {
			ipString[index++] = '.';
		} else {
			ipString[index] = 0;
		}
	}
}

/**
 * @brief Encode input data buffer as ASCII Base64 values
 *
 * REF: http://en.wikibooks.org/wiki/Algorithm_Implementation/Miscellaneous/Base64
 * This function will encode a buffer of binary data as an ASCII base64 for transmission
 * over a network packet.
 *
 * @param *data_buf input data buffer
 * @param dataLength the number of bytes in the input data buffer
 * @param *result pointer to the array to hold the result string
 * @param resultSize the number of bytes that can be stored in teh result buffer
 *
 * @return
 */
int `$INSTANCE_NAME`_Base64Encode( const void* data_buf, int dataLength,
                                   char* result, int resultSize)
{
    const char base64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const uint8_t *dat = (const uint8_t *)data_buf;
    int resultIndex = 0;
    int padCount = dataLength % 3;
    uint32_t n = 0;
    uint8_t n0 = 0;
    uint8_t n1 = 0;
    uint8_t n2= 0;
    uint8_t n3 = 0;
 
    // increment over the length of the string, three characters at a time
    for ( int x = 0; x < dataLength; x += 3 ) {
        // these three 8-bit (ASCII) characters become one 24-bit number
        n = dat[x] << 16;
  
        if ( ( x + 1 ) < dataLength ) {
            n += dat[ x + 1 ] << 8;
        }
  
        if ( ( x + 2 ) < dataLength ) {
            n += dat[ x + 2 ];
        }
  
        // this 24-bit number gets separated into four 6-bit numbers
        n0 = (uint8)(n >> 18) & 63;
        n1 = (uint8)(n >> 12) & 63;
        n2 = (uint8)(n >> 6) & 63;
        n3 = (uint8)n & 63;
 
        // if we have one byte available, then its encoding is spread out over two characters
        if ( resultIndex >= resultSize ) {
            return 0;   // indicate failure: buffer too small
        }
        
        result[resultIndex++] = base64chars[n0];
        
        if ( resultIndex >= resultSize ) {
            return 0;   // indicate failure: buffer too small
        }
        
        result[resultIndex++] = base64chars[n1];
 
        // if we have only two bytes available, then their encoding is spread out over three chars
        if ( ( x + 1 ) < dataLength ) {
            
            if( resultIndex >= resultSize ) {
                return 0;   // indicate failure: buffer too small
            }
            
            result[resultIndex++] = base64chars[n2];
        }
 
        // if we have all three bytes available, then their encoding is spread out over four characters
        if ( ( x + 2 ) < dataLength ) {
            if( resultIndex >= resultSize ) {
                return 0;   // indicate failure: buffer too small
            }
            result[resultIndex++] = base64chars[n3];
        }
    }
 
    // create and add padding that is required if we did not have a multiple of 3 number of characters available
    if (padCount > 0) 
    {
      for ( ; padCount < 3; padCount++ ) {
            if ( resultIndex >= resultSize ) {
                return 0;   // indicate failure: buffer too small
            }
            result[resultIndex++] = '=';
        } 
    }
    
    if( resultIndex >= resultSize ) {
        return 0;   // indicate failure: buffer too small
    }
    
    result[resultIndex] = 0;

    return 1;   // indicate success
}

/**
 * @brief decode a base64 encoded string in to corrisponding binary data
 *
 * REF: http://en.wikibooks.org/wiki/Algorithm_Implementation/Miscellaneous/Base64
 * This function will decode a packet of base64 encoded data and return the binary
 * equalivant data in the out buffer.  When using this function, the outLength is
 * passed as the maximum buffer size of the output buffer.  This value is modified
 * by the function to contain the actual length of the data decoded, thus a passed
 * value of 1024 for the maximum woudl have 35 after calling when 35 bytes were
 * decoded from the input string.
 *
 * @param *in pointer to the input buffer of base64 encoded data
 * @param inLen the length of data within the input buffer.
 * @param *out pointer to the array to hold the decoded data elements
 * @param *outLen pointer to the maximum & actual output length
 *
 * @retval 0 Result is ok
 * @retval 1 Buffer Overflow
 * @retval 2 Invalid data input
 */
int `$INSTANCE_NAME`_Base64Decode( char* in, int inLen, uint8_t* out, int* outLen )
{ 
    char *end = in + inLen;
    int buf = 1;
    int len = 0;
 	uint8_t c;
	
    while ( in < end ) {
		if ( (*in >= 'A') && (*in <= 'Z') )       { c = (*in) - 'A';	      }
		else if ( (*in >= 'a') && ( *in <= 'z') ) {	c = (*in) - 'a' + 26;     }
		else if ( (*in >= '0') && ( *in <= '9') ) { c = (*in) - '0' + 52;     }
		else if ( *in == '+' )                    { c = 62;                   }
		else if ( *in == '/' )                    {	c = 63;                   }
		else if ( *in == 9)                       { c = _DECODE64_WHITESPACE; }
		else if ( *in == '=' )                    { c = _DECODE64_EQUALS;     }
		else                                      { c = _DECODE64_INVALID;    }
 		++in;
		
        switch (c) {
        case _DECODE64_WHITESPACE: continue;   /* skip whitespace */
        case _DECODE64_INVALID:    return 2;   /* invalid input, return error */
        case _DECODE64_EQUALS:                 /* pad character, end of data */
            in = end;
            continue;
        default:
            buf = buf << 6 | c;
 
            // If the buffer is full, split it into bytes
            if (buf & 0x1000000) {
                if ((len += 3) > *outLen) return 1; // buffer overflow
                *out++ = buf >> 16;
                *out++ = buf >> 8;
                *out++ = buf;
                buf = 1;
            }   
        }
    }
 
    if (buf & 0x40000) {
        if ((len += 2) > *outLen) return 1; // buffer overflow
        *out++ = buf >> 10;
        *out++ = buf >> 2;
    } else if (buf & 0x1000) {
        if (++len > *outLen) return 1; // buffer overflow
        *out++ = buf >> 4;
    }
 
    // modify to reflect the actual output size
    *outLen = len;
    
    return 0;
}

/**
 * @brief
 *
 * @param 
 * @param
 * @param
 * @param
 *
 * @return
 */
uint32_t `$INSTANCE_NAME`_IPADDRESS( const uint8_t x1, const uint8_t x2,
                                        const uint8_t x3, const uint8_t x4 )
{
#if 0
	uint32_t adr;
	uint8_t* ptr = (uint8*)&adr;
	
	//ptr = (uint8*)&adr;
	ptr[0] = x1;
    ptr[1] = x2;
	ptr[2] = x3;
    ptr[3] = x4;
    
	return adr;
#else
    union {
        uint32_t adr;        
        uint8_t ptr[4];
    } u;
    
    u.ptr[0] = x1;
    u.ptr[1] = x2;
    u.ptr[2] = x3;
    u.ptr[3] = x4;
    
    return u.adr;
    
#endif
    
}

/**
 * Undefine the helper macros used to convert to/from ASCII-HEX from/to
 * binary.  This is done to prevent stepping on someone else's toes who
 * might have used the macro somewhere else.
 */
#undef _HEX2BIN
#undef _BIN2HEX
#undef _DECODE64_WHITESPACE
#undef _DECODE64_EQUALS
#undef _DECODE64_INVALID

/** @} */
/* [] END OF FILE */

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
#if !defined(W5500_H)
	#define W5500_H

#include <cytypes.h>
	
typedef struct {
	char gateway[16];
	char subnet[16];
	char mac[18];
	char ip[18];
	uint32 flags;
} w5500_config;

typedef struct {
	uint32 Gateway;
	uint32 subnet;
	uint8 MAC[6];
	uint32 ip;
	uint8 socketStatus[8];
} w5500_info;


#define W5500_FLG_SKT_UDP_MULTICAST_ENABLE       ( 0x80 )
#define W5500_FLG_SKT_MACRAW_MAC_FILT_ENABLE     ( 0x80 )
#define W5500_FLG_SKT_BLOCK_BROADCAST            ( 0x40 )
#define W5500_FLG_SKT_ND_ACK                     ( 0x20 )
#define W5500_FLG_SKT_UDP_IGMP_V1                ( 0x20 )
#define W5500_FLG_SKT_MACRAW_MULTICAST_BLOCK     ( 0x20 )
#define W5500_FLG_SKT_UDP_BLOCK_UNICAST          ( 0x10 )
#define W5500_FLG_SKT_MACRAW_IPV6_BLOCKING       ( 0x10 )
#define W5500_PROTO_CLOSED                       ( 0x00 )
#define W5500_PROTO_TCP                          ( 0x01 )
#define W5500_PROTO_UDP                          ( 0x02 )
#define W5500_PROTO_MACRAW                       ( 0x04 )


void w5500_Start( void );
void w5500_StartEx( w5500_config *config );

#endif

/* [] END OF FILE */

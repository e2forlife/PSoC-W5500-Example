#if !defined(W5500_H)
	#define W5500_H
/* ------------------------------------------------------------------------ */
/**
 * \defgroup e2forlife_w5500 W5500 Device Driver for PSoC
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
 * \file w5500.h
 * \author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Driver configuration, function prototypes, and chip register defines for
 * accessing the iW5500.
 */
/* ======================================================================== */

#include <cytypes.h>


/* Calculated memory size parameter to allocate more socket mem when using fewer sockets */
#define W5500_SOCKET_MEM         ( 2 )
/* Chip Configuration. Set to the maximum memory available for socket data */
#define W5500_CHIP_MEM           ( 16 )
#define W5500_CHIP_SOCKETS       ( 8 )
/* Chip Configuration (Calculated). This contains the calculated MAX sockets based on available memory */
#define W5500_MAX_SOCKETS        ( W5500_CHIP_MEM / W5500_SOCKET_MEM )
	
#define W5500_SOCKET_BAD(x)      ( (x >= W5500_MAX_SOCKETS)||(W5500_socketStatus[x] == W5500_SOCKET_AVAILALE) )


/* ------------------------------------------------------------------------ */
#define W5500_SOCKET_OPEN           ( 1 )
#define W5500_SOCKET_AVAILALE       ( 0 )

/* ------------------------------------------------------------------------ */
/* Macros & Function Defines */
/*
 * W5500 Register Map
 */
#define W5500_REG_MODE            ( 0x0000 )
#define W5500_REG_GAR             ( 0x0001 )
#define W5500_REG_SUBR            ( 0x0005 )
#define W5500_REG_SHAR            ( 0x0009 )
#define W5500_REG_SIPR            ( 0x000F )
#define W5500_REG_INTLEVEL        ( 0x0013 )
#define W5500_REG_IR              ( 0x0015 )
#define W5500_REG_IMR             ( 0x0016 )
#define W5500_REG_SIR             ( 0x0017 )
#define W5500_REG_SIMR            ( 0x0018 )
#define W5500_REG_RTR             ( 0x0019 )
#define W5500_REG_RCR             ( 0x001B )
#define W5500_REG_PTIMER          ( 0x001C )
#define W5500_REG_PMAGIC          ( 0x001D )
#define W5500_REG_PHAR            ( 0x001E )
#define W5500_REG_PSID            ( 0x0024 )
#define W5500_REG_PMRU            ( 0x0026 )
#define W5500_REG_UIPR            ( 0x0028 )
#define W5500_REG_UPORTR          ( 0x002C )
#define W5500_REG_PHYCFGR         ( 0x002E )
#define W5500_REG_VERSIONR        ( 0x0039 )

/* Socket Registers */
#define W5500_SREG_MR             ( 0x0000 )
#define W5500_SREG_CR             ( 0x0001 )
#define W5500_SREG_IR             ( 0x0002 )
#define W5500_SREG_SR             ( 0x0003 )
#define W5500_SREG_PORT           ( 0x0004 )
#define W5500_SREG_DHAR           ( 0x0006 )
#define W5500_SREG_DIPR           ( 0x000C )
#define W5500_SREG_DPORT          ( 0x0010 )
#define W5500_SREG_MSSR           ( 0x0012 )
#define W5500_SREG_TOS            ( 0x0015 )
#define W5500_SREG_TTL            ( 0x0016 )
#define W5500_SREG_RXBUF_SIZE     ( 0x001E )
#define W5500_SREG_TXBUF_SIZE     ( 0x001F )
#define W5500_SREG_TX_FSR         ( 0x0020 )
#define W5500_SREG_TX_RD          ( 0x0022 )
#define W5500_SREG_TX_WR          ( 0x0024 )
#define W5500_SREG_RX_RSR         ( 0x0026 ) 
#define W5500_SREG_RX_RD          ( 0x0028 )
#define W5500_SREG_RX_WR          ( 0x002A )
#define W5500_SREG_IMR            ( 0x002C )
#define W5500_FRAG                ( 0x002D )
#define W5500_KPALVTR             ( 0x002F )

/*
 * W5500 Commands
 */
#define W5500_CR_OPEN            ( 0x01 )
#define W5500_CR_LISTEN          ( 0x02 )
#define W5500_CR_CONNECT         ( 0x04 )
#define W5500_CR_DISCON          ( 0x08 )
#define W5500_CR_CLOSE           ( 0x10 )
#define W5500_CR_SEND            ( 0x20 )
#define W5500_CR_SEND_MAC        ( 0x21 )
#define W5500_CR_SEND_KEEP       ( 0x22 )
#define W5500_CR_RECV            ( 0x40 )

/*
 * W5500 Interrupt Flags
 */
#define W5500_IR_CON              ( 0x01 )
#define W5500_IR_DISCON           ( 0x02 )
#define W5500_IR_RECV             ( 0x04 )
#define W5500_IR_TIMEOUT          ( 0x08 )
#define W5500_IR_SEND_OK          ( 0x10 )

/*
 * Socket Status
 */
#define W5500_SR_CLOSED         ( 0x00 )
#define W5500_SR_INIT           ( 0x13 )
#define W5500_SR_LISTEN         ( 0x14 )
#define W5500_SR_ESTABLISHED    ( 0x17 )
#define W5500_SR_CLOSE_WAIT     ( 0x1C )
#define W5500_SR_UDP            ( 0x22 )
#define W5500_SR_MACRAW         ( 0x42 )

/*
 * timeout used to prevent command acceptance deadlock
 */
#define W5500_CMD_TIMEOUT         ( 125 )

/* Block select regions of the W5500 */
#define W5500_BLOCK_COMMON        (0x00)
#define W5500_BLOCK_S0_REG        (0x08)
#define W5500_BLOCK_S0_TXB        (0x10)
#define W5500_BLOCK_S0_RXB        (0x18)
#define W5500_BLOCK_S1_REG        (0x28)
#define W5500_BLOCK_S1_TXB        (0x30)
#define W5500_BLOCK_S1_RXB        (0x38)
#define W5500_BLOCK_S2_REG        (0x48)
#define W5500_BLOCK_S2_TXB        (0x50)
#define W5500_BLOCK_S2_RXB        (0x58)
#define W5500_BLOCK_S3_REG        (0x68)
#define W5500_BLOCK_S3_TXB        (0x70)
#define W5500_BLOCK_S3_RXB        (0x78)
#define W5500_BLOCK_S4_REG        (0x88)
#define W5500_BLOCK_S4_TXB        (0x90)
#define W5500_BLOCK_S4_RXB        (0x98)
#define W5500_BLOCK_S5_REG        (0xA8)
#define W5500_BLOCK_S5_TXB        (0xB0)
#define W5500_BLOCK_S5_RXB        (0xB8)
#define W5500_BLOCK_S6_REG        (0xC8)
#define W5500_BLOCK_S6_TXB        (0xD0)
#define W5500_BLOCK_S6_RXB        (0xD8)
#define W5500_BLOCK_S7_REG        (0xE8)
#define W5500_BLOCK_S7_TXB        (0xF0)
#define W5500_BLOCK_S7_RXB        (0xF8)

/* 
 * MACRO functions to calculate the socket region block selects
 * from the socket number.  These are helpful when working with
 * the socket based commands.
 */
#define W5500_SOCKET_BASE(s)                     (( (s*4) + 1 )<<3)
#define W5500_TX_BASE(s)                         (( (s*4) + 2 )<<3)
#define W5500_RX_BASE(s)                         (( (s*4) + 3 )<<3)

/* Socket Configuration FLags */
#define W5500_FLG_SKT_UDP_MULTICAST_ENABLE       ( 0x80 )
#define W5500_FLG_SKT_MACRAW_MAC_FILT_ENABLE     ( 0x80 )
#define W5500_FLG_SKT_BLOCK_BROADCAST            ( 0x40 )
#define W5500_FLG_SKT_ND_ACK                     ( 0x20 )
#define W5500_FLG_SKT_UDP_IGMP_V1                ( 0x20 )
#define W5500_FLG_SKT_MACRAW_MULTICAST_BLOCK     ( 0x20 )
#define W5500_FLG_SKT_UDP_BLOCK_UNICAST          ( 0x10 )
#define W5500_FLG_SKT_MACRAW_IPV6_BLOCKING       ( 0x10 )
/* Socket protocol configurations */
#define W5500_PROTO_CLOSED                       ( 0x00 )
#define W5500_PROTO_TCP                          ( 0x01 )
#define W5500_PROTO_UDP                          ( 0x02 )
#define W5500_PROTO_MACRAW                       ( 0x04 )

#define W5500_TXRX_FLG_WAIT                      ( 0x01 )

/* ------------------------------------------------------------------------ */
void w5500_Send(uint16 offset, uint8 block_select, uint8 write, uint8 *buffer, uint16 len);

cystatus w5500_Start( void );
cystatus w5500_StartEx( const char *gateway, const char *subnet, const char *mac, const char *ip );
cystatus w5500_Init( uint8* gateway, uint8* subnet, uint8 *mac, uint8* ip );

uint16 w5500_RxDataReady( uint8 socket );
uint16 w5500_TxBufferFree( uint8 socket );
uint8 w5500_SocketOpen( uint16 port, uint8 flags );
uint8 w5500_SocketClose( uint8 sock, uint8 discon );
uint8 w5500_SocketDisconnect( uint8 sock );
uint8 w5500_SocketSendComplete( uint8 socket );
uint8 w5500_ExecuteSocketCommand(uint8 socket, uint8 cmd );

cystatus w5500_TcpConnected( uint8 sock );
uint8 w5500_TcpOpenClient( uint16 port, uint32 remote_ip, uint16 remote_port );
uint8 w5500_TcpOpenServer(uint16 port);
cystatus w5500_TcpWaitForConnection( uint8 socket );
uint16 w5500_TcpSend( uint8 socket, uint8* buffer, uint16 len, uint8 flags);
void w5500_TcpPrint(uint8 socket, const char *string );
uint16 w5500_TcpReceive(uint8 socket, uint8* buffer, uint16 len, uint8 flags);
char w5500_TcpGetChar( uint8 socket );
int w5500_TcpGetLine( uint8 socket, char *buffer );

#endif
/** @} */
/* [] END OF FILE */

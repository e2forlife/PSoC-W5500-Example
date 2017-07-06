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
 * \file tcp.c
 * \author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Implementation of the TCP protocol using the internal WizNET protocol stack
 * in the iEthernet devices.
 */

#include <cytypes.h>
#include <CyLib.h>
#include <string.h>

#include "`$INSTANCE_NAME`.h"
#include "`$INSTANCE_NAME`_udp.h"
#include "`$INSTANCE_NAME`_socket.h"

extern uint8_t `$INSTANCE_NAME`_socketStatus[`$INSTANCE_NAME`_MAX_SOCKETS];

/**
 * @brief
 *
 * @param 
 *
 * @return
 */
uint8_t `$INSTANCE_NAME`_UdpOpen( uint16_t port )
{
	uint8_t socket = 0;
    uint8_t status = `$INSTANCE_NAME`_SR_CLOSED;
    uint8_t tries = 0;
	
	do {
		socket = `$INSTANCE_NAME`_SocketOpen(port, `$INSTANCE_NAME`_PROTO_UDP);
	
		if (socket < `$INSTANCE_NAME`_MAX_SOCKETS) {
            `$INSTANCE_NAME`_Read(`$INSTANCE_NAME`_SREG_SR,
                                  `$INSTANCE_NAME`_SOCKET_BASE(socket),
                                  &status, 1);
			if (status != `$INSTANCE_NAME`_SR_UDP) {
				`$INSTANCE_NAME`_SocketClose(socket,0);
				socket = 0xFF;
			}
		}
		++tries;
	} while ( (tries < 5) && ( status != `$INSTANCE_NAME`_SR_UDP ) );
	
	return socket;
}

/**
 * @brief Send a block of data using UDP.
 *
 * @param 
 * @param
 * @param
 * @param
 * @param
 * @param
 *
 * @return
 */
uint16_t `$INSTANCE_NAME`_UdpSend( uint8_t socket, uint32_t ip, uint16_t port,
                                   uint8_t *buffer, uint16_t len, uint8_t flags )
{
	uint16_t tx_length = `$INSTANCE_NAME`_GetTxLength(socket,len,flags);
	
	port = CYSWAP_ENDIAN16(port); // fix endian-ness
    
	// setup destination information
    `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_DIPR,
                            `$INSTANCE_NAME`_SOCKET_BASE(socket),
                            (uint8*)&ip, 4 );
    `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_DPORT,
                            `$INSTANCE_NAME`_SOCKET_BASE(socket),
                            (uint8*)&port, 2 );
	`$INSTANCE_NAME`_WriteTxData(socket, buffer, tx_length, flags);
	
	return tx_length;
}

/**
 * @brief
 *
 * @param 
 * @param
 * @param
 * @param
 * @param
 *
 * @return
 */
uint16_t `$INSTANCE_NAME`_UdpReceive( uint8_t socket, uint8_t *header,
                                      uint8_t *buffer, uint16_t len, uint8_t flags )
{
	uint16_t rx_size = 0;
    uint16_t bytes = 0;
    uint16_t ptr = 0;
	
	// request the length of the data block available for reading, but, add
	// the header size (8 bytes) to the length of data requested to account
	// for the header sitting in the Rx Buffers.
	do {
		rx_size = `$INSTANCE_NAME`_RxDataReady( socket );
	} while ( ( 8 > rx_size ) && ( flags & `$INSTANCE_NAME`_TXRX_FLG_WAIT ) );
	
	// if there is data to read from the buffer...
	if ( 7 < rx_size ) {
		// calculate the number of bytes to receive using the available data
		// and the requested length of data.
		bytes = ( rx_size > len ) ? len : rx_size;
		// Read the starting memory pointer address, and endian correct
        `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_RX_RD,
                               `$INSTANCE_NAME`_SOCKET_BASE(socket),
                               (uint8*)&ptr, 2 );
		ptr = CYSWAP_ENDIAN16( ptr );
		// Read the UDP header block from the memory
        `$INSTANCE_NAME`_Read( ptr,`$INSTANCE_NAME`_RX_BASE(socket), header, 8);
		ptr += 8;
		// read the number of bytes to read from the UDP header
		bytes = header[6];
		bytes = ( bytes << 8 ) + header[7];
		
		// Retrieve the length of data from the received UDP packet, starting
		// right after the end of the packet header.
        `$INSTANCE_NAME`_Read( ptr, `$INSTANCE_NAME`_RX_BASE(socket), buffer, bytes);
		// Calculate the new buffer pointer location, endian correct, and
		// update the pointer register within the W5500 socket registers
		ptr += bytes;
		ptr = CYSWAP_ENDIAN16( ptr );
        `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_RX_RD,
                                `$INSTANCE_NAME`_SOCKET_BASE(socket),
                                (uint8*)&ptr, 2 );
		// when all of the available data was read from the message, execute
		// the receive command
		`$INSTANCE_NAME`_ExecuteSocketCommand( socket, `$INSTANCE_NAME`_CR_RECV );
		
	}
	
	return bytes;
}

/** @todo Open Multi-cast socket */

/** @todo Send Multi-cast data */

/** @} */
/* [] END OF FILE */

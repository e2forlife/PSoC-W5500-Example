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
 * \file tcp.c
 * \author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Implementation of the TCP protocol using the internal WizNET protocol stack
 * in the iEthernet devices.
 */
/* ======================================================================== */
#include <cytypes.h>
#include <cylib.h>
#include <string.h>

#include "w5500.h"

extern w5500_info w5500_ChipInfo;

/* ------------------------------------------------------------------------ */
/**
 * \brief Read the status of the socket and check to see if a connection is established
 * \param sock (uint8) the socket number to which status shoudl be checked
 * \returns 0 Socket is not yet established
 * \returns 0xFF Socket is not open
 * \returns 0x80 Socket Timeout
 * \returns 0x01 the socket connection is established
 */
uint8 w5500_TcpConnected( uint8 sock )
{
	uint8 status;
	
	if (sock > 7) return 0;
	
	if (w5500_ChipInfo.socketStatus[sock] == W5500_SOCKET_AVAILALE) {
		return CYRET_INVALID_STATE;
	}
	
	w5500_Send(W5500_SREG_SR,W5500_SOCKET_BASE(sock),0,&status, 1);
	if (status == W5500_SR_ESTABLISHED) {
		return CYRET_SUCCESS;
	}
	else {
		w5500_Send(W5500_SREG_IR, W5500_SOCKET_BASE(sock),0,&status, 1);
		if ( (status & W5500_IR_TIMEOUT) != 0) {
			status = 0xFF;
			w5500_Send(W5500_SREG_IR, W5500_SOCKET_BASE(sock),1,&status,1);
			return CYRET_TIMEOUT;
		}
	}
	return CYRET_STARTED;
}
/* ------------------------------------------------------------------------ */
/**
 * \brief Open a socket and set protocol to TCP mode
 * \param port (uint16) the port number on which to open the socket
 * \param remote_ip (uint32) The remote IP address to connect
 * \param remote_port (uint16) the port on the remote server to connect
 * \returns 0xFF Unable to open the socket
 * \returns uint8 socket number allocated for this socket
 *
 *
 */
uint8 w5500_TcpOpenClient( uint16 port, uint32 remote_ip, uint16 remote_port )
{
	uint8 socket;
	uint32 timeout;
	uint8 ir;
	uint8 rCfg[6];
	
	/* open the socket using the TCP mode */
	socket = w5500_SocketOpen( port, W5500_PROTO_TCP );

	/*
	 * 2.0 Patch: retun immediately upon the detection of a socket that is not open
	 */
	if (W5500_SOCKET_BAD(socket) ) return 0xFF;
	if ( (remote_ip != 0xFFFFFFFF) && (remote_ip != 0) ) {
		/*
		 * a valid socket was opened, so now we can use the socket handle to
		 * open the client connection to the specified server IP.
		 * So, this builds a configuration packet to send to the device
		 * to reduce the operations executed at one time (lower overhead)
		 */
		remote_port = CYSWAP_ENDIAN16(remote_port);
		memcpy((void*)&rCfg[0], (void*)&remote_ip, 4);
		memcpy((void*)&rCfg[4],(void*)&remote_port,2);
		
		/*
		 * Blast out the configuration record all at once to set up the IP and
		 * port for the remote connection.
		 */
		w5500_Send(W5500_SREG_DIPR, W5500_SOCKET_BASE(socket),1,&rCfg[0], 6);

		/*
		 * Execute the connection to the remote server and check for errors
		 */
		if (w5500_ExecuteSocketCommand(socket, W5500_CR_CONNECT) == 0) {
			timeout = 0;
			/* wait for the socket connection to the remote host is established */
			do {
				CyDelay(1);
				++timeout;
				w5500_Send(W5500_SREG_IR, W5500_SOCKET_BASE(socket), 0, &ir, 1);
				if ( (ir & 0x08) != 0 ) {
					/* internal chip timeout occured */
					timeout = 3000;
				}				
			}
			while ( ((ir&0x01) == 0)  && (timeout < 3000) );
		}
		else {
			w5500_SocketClose(socket,0);
			socket = 0xFF;
		}
	}
	return socket;
}
/* ------------------------------------------------------------------------ */
/**
 * \brief Open a TCP server socket using a specified port
 * \param port The port number to assign to the socket
 * \returns uint8 socket number for open server socket
 * \returns 0xFF Socket error, cannot open socket.
 *
 * _TcpOpenServer opens a socket with the TCP protocol on the port specified
 * by the parameter (port).  After opening the socket, _TcpOpenServer will
 * issue a LISTEN command to the W5500 to initiate a server. The server will
 * connect when a valid SYN packet is received.
 * \sa _SocketOpen
 */
uint8 w5500_TcpOpenServer(uint16 port)
{
	uint8 socket;
	uint8 status;
	
	/* open the socket using the TCP mode */
	socket = w5500_SocketOpen( port, W5500_PROTO_TCP );

	/*
	 * 2.0 Patch: retun immediately upon the detection of a socket that is not open
	 */
	if ( W5500_SOCKET_BAD(socket)) return 0xFF;
	
	w5500_Send(W5500_SREG_SR,W5500_SOCKET_BASE(socket),0,&status,1);
	if (status != W5500_SR_INIT) {
		/*
		 * Error opening socket
		 */
		w5500_SocketClose(socket,0);
	}
	else if (w5500_ExecuteSocketCommand( socket, W5500_CR_LISTEN) != CYRET_SUCCESS) {
		w5500_SocketClose( socket, 0);
		socket = 0xFF;
	}
	
	return socket;
}
/* ------------------------------------------------------------------------ */
/**
 * \brief suspend operation while waiting for a connection to be established
 * \param socket (uint8) Socket handle for an open socket.
 *
 * _TcpWaitForConnection will poll the W5500 and wait until a connection has
 * been established.  Note that this is a BLOCKING call and will deadlock your
 * control loop if your application waits for long periods between connections.
 * Use _TcpConnected for non-blocking scans.
 * \sa _TcpConnected
 */ 
uint8 w5500_TcpWaitForConnection( uint8 socket )
{
	uint8 status;

	/*
	 * If the socket is invalid or not yet open, return a non-connect result
	 * to prevent calling functions and waiting for the timeout for sockets
	 * that are not yet open
	 */
	if (W5500_SOCKET_BAD(socket)) return CYRET_BAD_PARAM;
	/*
	 * Wait for the connectino to be established, or a timeout on the connection
	 * delay to occur.
	 */
	do {
		CyDelay(10);
		w5500_Send(W5500_SREG_SR,W5500_SOCKET_BASE(socket),0,&status, 1);
	}
	while ( status == W5500_SR_LISTEN );
		
	return CYRET_SUCCESS;
}
/* ------------------------------------------------------------------------ */
uint16 w5500_TcpSend( uint8 socket, uint8* buffer, uint16 len, uint8 flags)
{
	uint16 tx_length;
	uint16 max_packet;
	uint8 buf_size;
	uint16 ptr;
	uint8 result;
	
	if (W5500_SOCKET_BAD(socket) ) return 0;
	
	tx_length = w5500_TxBufferFree( socket );
	if ( (tx_length < len ) && ((flags&W5500_TXRX_FLG_WAIT) != 0) ) {
		/* 
		 * there is not enough room in the buffer, but the caller requested
		 * this to block until there was free space. So, check the memory
		 * size to determine if the tx buffer is big enough to handle the
		 * data block without fragmentation.
		 */
		w5500_Send(W5500_SREG_TXBUF_SIZE, W5500_SOCKET_BASE(socket),0,&buf_size,1);
		max_packet = (buf_size == 0)? 0 : (0x400 << (buf_size-1));
		/*
		 * now that we know the max buffer size, if it is smaller than the
		 * requested transmit lenght, we have an error, so return 0
		 */
		if (max_packet < len ) return 0;
		/* otherwise, we will wait for the room in the buffer */
		do {
			tx_length = w5500_TxBufferFree( socket );
		}
		while ( tx_length < len );
	}
	else {
		tx_length = len;
	}
	/*
	 * The length of the Tx data block has now been determined, and can be
	 * copied in to the W5500 buffer memory. First read the pointer, then
	 * write data from the pointer forward, lastly update the pointer and issue
	 * the SEND command.
	 */
	w5500_Send( W5500_SREG_TX_WR, W5500_SOCKET_BASE(socket),0,(uint8*)&ptr,2);
	ptr = CYSWAP_ENDIAN16( ptr );
	w5500_Send( ptr, W5500_TX_BASE(socket),1,buffer,tx_length);
	ptr += tx_length;
	ptr = CYSWAP_ENDIAN16( ptr );
	w5500_Send(W5500_SREG_TX_WR, W5500_SOCKET_BASE(socket),1,(uint8*)&ptr,2);
	
	w5500_ExecuteSocketCommand( socket, W5500_CR_SEND );
	
	if ( (flags & W5500_TXRX_FLG_WAIT) != 0) {
		/*
		 * block until send is complete
		 */
		do {
			CyDelay(1);
			result = w5500_SocketSendComplete(socket);
		}
		while( (result != CYRET_FINISHED) && (result != CYRET_CANCELED) );
	}
	
	return tx_length;
}
/* ------------------------------------------------------------------------ */
void w5500_TcpPrint(uint8 socket, const char *string )
{
	uint16 length;
	
	length = strlen(string);
	w5500_TcpSend(socket, (uint8*) string,length, 0);
}
/** @} */
/* [] END OF FILE */

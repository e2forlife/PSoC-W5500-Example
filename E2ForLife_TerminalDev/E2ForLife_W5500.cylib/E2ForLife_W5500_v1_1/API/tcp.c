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
 * @file tcp.c
 * @author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * Implementation of the TCP protocol using the internal WizNET protocol stack
 * in the iEthernet devices.
 */

#include <cytypes.h>
#include <CyLib.h>
#include <string.h>

#include "`$INSTANCE_NAME`.h"
#include "`$INSTANCE_NAME`_socket.h"
#include "`$INSTANCE_NAME`_tcp.h"

extern uint8_t `$INSTANCE_NAME`_socketStatus[`$INSTANCE_NAME`_MAX_SOCKETS];

/**
 * @brief Read the status of the socket and check to see if a connection is established.
 *
 *
 *
 * @param sock (uint8) the socket number to which status shoudl be checked.
 *
 * @returns 0 Socket is not yet established
 * @returns 0xFF Socket is not open
 * @returns 0x80 Socket Timeout
 * @returns 0x01 the socket connection is established
 */
cystatus `$INSTANCE_NAME`_TcpConnected( uint8_t socket )
{
    uint8_t status = 0;
	
    if ( `$INSTANCE_NAME`_SOCKET_BAD(socket) ) {
        return CYRET_BAD_PARAM;
    }
    `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_SR,
                           `$INSTANCE_NAME`_SOCKET_BASE(socket),
                           &status, 1 );
    if ( `$INSTANCE_NAME`_SR_ESTABLISHED == status ) {
        return CYRET_SUCCESS;
    } else {
        `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_IR,
                               `$INSTANCE_NAME`_SOCKET_BASE(socket),
                               &status, 1 );
        if ( 0 != ( status & `$INSTANCE_NAME`_IR_TIMEOUT ) ) {
            status = 0xFF;
            `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_IR,
                                    `$INSTANCE_NAME`_SOCKET_BASE(socket),
                                    &status, 1);
            return CYRET_TIMEOUT;
        }
    }
    
    return CYRET_STARTED;
}

/**
 * @brief Open a socket and set protocol to TCP mode.
 *
 *
 *
 * @param port (uint16) the port number on which to open the socket
 * @param remote_ip (uint32) The remote IP address to connect
 * @param remote_port (uint16) the port on the remote server to connect
 *
 * @returns 0xFF Unable to open the socket
 * @returns uint8 socket number allocated for this socket
 */
uint8_t `$INSTANCE_NAME`_TcpOpenClient( uint16_t port, uint32_t remote_ip,
                                        uint16_t remote_port )
{
    uint32_t timeout = 0;
    uint8_t ir = 0;
    uint8_t rCfg[6] = {0};
    
    // open the socket using the TCP mode
    uint8_t socket  = `$INSTANCE_NAME`_SocketOpen( port, `$INSTANCE_NAME`_PROTO_TCP );

    // 2.0 Patch: retun immediately upon the detection of a socket that is not open
    if ( `$INSTANCE_NAME`_SOCKET_BAD(socket) ) {
        return 0xFF;
    }
    
    if ( ( remote_ip != 0xFFFFFFFF ) && ( remote_ip != 0 ) ) {
        // a valid socket was opened, so now we can use the socket handle to
        // open the client connection to the specified server IP.
        // So, this builds a configuration packet to send to the device
        // to reduce the operations executed at one time (lower overhead)
        remote_port = CYSWAP_ENDIAN16(remote_port);
        memcpy((void*)&rCfg[0], (void*)&remote_ip, 4);
        memcpy((void*)&rCfg[4], (void*)&remote_port, 2);

        // Blast out the configuration record all at once to set up the IP and
        // port for the remote connection.
        `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_DIPR,
                                `$INSTANCE_NAME`_SOCKET_BASE(socket),
                                &rCfg[0], 6 );
        // Execute the connection to the remote server and check for errors
        if ( CYRET_SUCCESS == `$INSTANCE_NAME`_ExecuteSocketCommand( socket,
                                                                     `$INSTANCE_NAME`_CR_CONNECT ) ) {
                timeout = 0;
                // wait for the socket connection to the remote host is established
                do {
                    CyDelay(1);
                    ++timeout;
                    `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_IR,
                                           `$INSTANCE_NAME`_SOCKET_BASE(socket),
                                           &ir, 1 );
                    if ( 0 != ( ir & 0x08 ) ) {
                            // internal chip timeout occured
                            timeout = 3000;
                    }
                    
                } while ( ( 0 == ( ir&0x01 ) )  && ( timeout < 3000 ) );
        } else {
            `$INSTANCE_NAME`_SocketClose( socket, 0 );
            socket = 0xFF;
        }
    }
    
    return socket;
}

/**
 * @brief Open a TCP server socket using a specified port
 *
 * `$INSTANCE_NAME`_TcpOpenServer opens a socket with the TCP protocol on the 
 * port specified by the parameter (port).  After opening the socket,
 * `$INSTANCE_NAME`_TcpOpenServer will issue a LISTEN command to the W5500 to
 * initiate a server. The server will connect when a valid SYN packet is received.
 *
 * @sa _SocketOpen
 *
 * @param port The port number to assign to the socket
 *
 * @returns uint8 socket number for open server socket
 * @returns 0xFF Socket error, cannot open socket.
 */
uint8_t `$INSTANCE_NAME`_TcpOpenServer( uint16_t port )
{
	uint8_t status = 0;
    
    // open the socket using the TCP mode
    uint8_t socket = `$INSTANCE_NAME`_SocketOpen( port,
                                                  `$INSTANCE_NAME`_PROTO_TCP );

	// 2.0 Patch: retun immediately upon the detection of a socket that is not open
    if ( `$INSTANCE_NAME`_SOCKET_BAD(socket) ) {
        return 0xFF;
    }
    `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_SR,
                           `$INSTANCE_NAME`_SOCKET_BASE(socket),
                           &status, 1 );

    if ( `$INSTANCE_NAME`_SR_INIT != status ) {
        `$INSTANCE_NAME`_SocketClose( socket, 0 ); // Error opening socket
    } else if ( CYRET_SUCCESS != `$INSTANCE_NAME`_ExecuteSocketCommand( socket,
                                                                        `$INSTANCE_NAME`_CR_LISTEN ) ) {
        `$INSTANCE_NAME`_SocketClose( socket, 0 );
        socket = 0xFF;
    }
	
    return socket;
}

/**
 * @brief Suspend operation while waiting for a connection to be established.
 *
 * _TcpWaitForConnection will poll the W5500 and wait until a connection has
 * been established.  Note that this is a BLOCKING call and will deadlock your
 * control loop if your application waits for long periods between connections.
 * Use _TcpConnected for non-blocking scans.
 * @sa _TcpConnected
 *
 * @param socket (uint8) Socket handle for an open socket.
 *
 * @return
 */ 
cystatus `$INSTANCE_NAME`_TcpWaitForConnection( uint8_t socket )
{
	uint8_t status = 0;

    // If the socket is invalid or not yet open, return a non-connect result
    // to prevent calling functions and waiting for the timeout for sockets
    // that are not yet open
    if ( `$INSTANCE_NAME`_SOCKET_BAD(socket) ) {
        return CYRET_BAD_PARAM;
    }
    
    // Wait for the connection to be established,
    // or a timeout on the connection delay to occur.
    do {
        CyDelay(10);
        `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_SR,
                               `$INSTANCE_NAME`_SOCKET_BASE(socket),
                               &status, 1 );
    } while ( `$INSTANCE_NAME`_SR_LISTEN == status );
		
    return CYRET_SUCCESS;
}

/**
 * @brief Generic transmission of a data block using TCP.
 *
 * `$INSTANCE_NAME`_TcpSend transmits a block of generic data using TCP through a socket.
 * the connection must have been previously established in order for the
 * the function to operate properly, otherwise, no data will be transmitted.
 *
 * @param socket the socket that will be used to send the data
 * @param *buffer the array of data to be sent using TCP
 * @param len the length of data to send from the buffer
 * @param flags control flags for controlling options for transmission
 *
 * @return
 */
uint16_t `$INSTANCE_NAME`_TcpSend( uint8_t socket, uint8_t* buffer,
                                   uint16_t len, uint8_t flags )
{
    uint16_t tx_length = 0;
    uint16_t max_packet = 0;
    uint16_t ptr = 0;
    uint8_t buf_size = 0;
    uint8_t result = 0;
	
    if ( `$INSTANCE_NAME`_SOCKET_BAD(socket) ) {
        return 0;
    }
	
    tx_length = `$INSTANCE_NAME`_TxBufferFree( socket );
    
    if ( ( tx_length < len ) && ( 0 != ( flags & `$INSTANCE_NAME`_TXRX_FLG_WAIT ) ) ) {
        
        // there is not enough room in the buffer, but the caller requested
        // this to block until there was free space. So, check the memory
        // size to determine if the tx buffer is big enough to handle the
        // data block without fragmentation.
        `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_TXBUF_SIZE,
                               `$INSTANCE_NAME`_SOCKET_BASE(socket),
                               &buf_size, 1);
        max_packet = ( buf_size == 0 ) ? 0 : ( 0x400 << ( buf_size - 1 ) );
        
        // now that we know the max buffer size, if it is smaller than the
        // requested transmit lenght, we have an error, so return 0
        if ( max_packet < len ) {
            return 0;
        }
        // otherwise, we will wait for the room in the buffer
        do {
                tx_length = `$INSTANCE_NAME`_TxBufferFree( socket );
        } while ( tx_length < len );

    } else {
        tx_length = len;
    }
    
    // The length of the Tx data block has now been determined, and can be
    // copied in to the W5500 buffer memory.
    // First read the pointer, then write data from the pointer forward,
    // lastly update the pointer and issue the SEND command.
    `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_TX_WR,
                           `$INSTANCE_NAME`_SOCKET_BASE(socket),
                           (uint8*)&ptr, 2 );
    ptr = CYSWAP_ENDIAN16( ptr );
    `$INSTANCE_NAME`_Write( ptr, `$INSTANCE_NAME`_TX_BASE(socket),
                            buffer, tx_length);
    ptr += tx_length;
    ptr = CYSWAP_ENDIAN16( ptr );
    `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_TX_WR,
                            `$INSTANCE_NAME`_SOCKET_BASE(socket),
                            (uint8*)&ptr, 2 );
    `$INSTANCE_NAME`_ExecuteSocketCommand( socket, `$INSTANCE_NAME`_CR_SEND );
	
    if ( 0 != ( flags & `$INSTANCE_NAME`_TXRX_FLG_WAIT ) ) {
        // block until send is complete
        do {
            CyDelay(1);
            result = `$INSTANCE_NAME`_SocketSendComplete( socket );
        } while ( ( CYRET_FINISHED != result ) && ( CYRET_CANCELED != result ) );
    }
	
    return tx_length;
}

/**
 * @brief Send an ASCII String using TCP.
 *
 * `$INSTANCE_NAME`_TcpPrint is a wrapper for the TcpSend function to simplify the
 * transmission of strings, and prompts over the open connection using TCP.
 * @sa `$INSTANCE_NAME`_TcpSend
 *
 * @param socket the socket to use for sending the data
 * @param *string ASCII-Z String to send using TCP
 *
 * @return
 */
void `$INSTANCE_NAME`_TcpPrint( uint8_t socket, const char* string )
{
    uint16_t length = strlen( string );
    `$INSTANCE_NAME`_TcpSend( socket, (uint8*) string, length, 0 );
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
uint16_t `$INSTANCE_NAME`_TcpReceive( uint8_t socket, uint8_t* buffer,
                                      uint16_t len, uint8_t flags )
{
    uint16_t rx_size = 0;
    uint16_t ptr = 0;
    uint16_t bytes = 0;
	
    // when there is a bad socket, just return 0 bys no matter what.
    if ( `$INSTANCE_NAME`_SOCKET_BAD(socket) ) {
        return 0;
    }

    // Otherwise, read the number of bytes waiting to be read.  When the byte
    // count is less than the requested bytes, wait for them to be available
    // when the wait flag is set, otherwise, just read the waiting data once.
    do {
        rx_size = `$INSTANCE_NAME`_RxDataReady( socket );
    } while ( ( rx_size < len ) && ( flags & `$INSTANCE_NAME`_TXRX_FLG_WAIT ) );
	
    // When data is available, begin processing the data
    if ( 0 < rx_size ) {
        // calculate the number of bytes to receive using the available data
        // and the requested length of data.
        bytes = ( rx_size > len ) ? len : rx_size;
        // Read the starting memory pointer address, and endian correct
        `$INSTANCE_NAME`_Read( `$INSTANCE_NAME`_SREG_RX_RD,
                               `$INSTANCE_NAME`_SOCKET_BASE(socket),
                               (uint8*)&ptr, 2 );
        ptr = CYSWAP_ENDIAN16( ptr );
        // Retrieve the data bytes from the W5500 buffer
        `$INSTANCE_NAME`_Read( ptr, `$INSTANCE_NAME`_RX_BASE(socket),
                               buffer, bytes);
        // Calculate the new buffer pointer location, endian correct, and
        // update the pointer register within the W5500 socket registers
        ptr += bytes;
        ptr = CYSWAP_ENDIAN16( ptr );
        `$INSTANCE_NAME`_Write( `$INSTANCE_NAME`_SREG_RX_RD,
                                `$INSTANCE_NAME`_SOCKET_BASE(socket),
                                (uint8*)&ptr, 2 );
        // when all of the available data was read from the message, execute the receive command
        `$INSTANCE_NAME`_ExecuteSocketCommand( socket, `$INSTANCE_NAME`_CR_RECV );
    }
    
    return bytes;
}

/**
 * @brief TCP Get char.
 *
 * @param 
 *
 * @return
 */
char `$INSTANCE_NAME`_TcpGetChar( uint8_t socket )
{
    char ch = 0;
    uint16_t len = 0;
	
    do {
        len = `$INSTANCE_NAME`_TcpReceive( socket, (uint8*)&ch, 1, 0 );
    } while ( len < 1 );
    
    return ch;
}

/**
 * @brief TCP Get line.
 *
 * @param 
 * @param 
 *
 * @return
 */
int `$INSTANCE_NAME`_TcpGetLine( uint8_t socket, char *buffer )
{
    char ch = 0;
    int idx = 0;
	
    do {
        ch = `$INSTANCE_NAME`_TcpGetChar( socket );
        if ( ( ch != '\r' ) && ( ch != '\n' ) ) {
                if ( ( ch == '\b' ) || ( ch == 127 ) ) { // 127 == DEL
                    buffer[idx] = 0;
                    idx = ( idx == 0 ) ? 0 : idx - 1;
                } else {
                    buffer[idx++] = ch;
                    buffer[idx] = 0;
#if 0
                    buffer[idx] = ch;
                    idx++;
                    buffer[idx] = 0;
#endif
                }
        }
    } while ( ( ch != '\r' ) && ( ch != '\n' ) );
    
    buffer[idx] = 0;
	
    return idx;
}

/** @} */
/* [] END OF FILE */

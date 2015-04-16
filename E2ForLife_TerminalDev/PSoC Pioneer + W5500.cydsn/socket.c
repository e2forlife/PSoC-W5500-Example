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
#include <cytypes.h>
#include <cylib.h>

#include "w5500.h"

extern w5500_info w5500_ChipInfo;

/* ------------------------------------------------------------------------ */
/**
 * \brief send a socket command
 * \param socket (uint8) the socket to command
 * \param cmd (uint8) the command to issue to the socket.
 *
 * This function writes teh command register then waits for the command
 * register to return a 0. then it returns the last read value.
 */
uint8 w5500_ExecuteSocketCommand(uint8 socket, uint8 cmd )
{
	uint8 result;
	uint32 timeout;
	
	if (socket >= W5500_MAX_SOCKETS) return CYRET_INVALID_OBJECT;
	
	w5500_Send(W5500_SREG_CR, W5500_SOCKET_BASE(socket), 1, &cmd, 1);
	timeout = 0;
	do {
		w5500_Send( W5500_SREG_CR, W5500_SOCKET_BASE(socket), 0, &result, 1);
		if (result != 0) {
			CyDelay(1);
			++timeout;
		}
	}
	while ( (result != 0) && (timeout < W5500_CMD_TIMEOUT) );
	
	result = (result == 0)? CYRET_SUCCESS : CYRET_TIMEOUT;
	return result;
}
/* ------------------------------------------------------------------------ */
/**
 * \brief Open and initialize a socket for use
 * \param port (uint16) the por tnumbe ron which the socket will be open
 * \param flags (uint8) Protocol and setting information for the socket mode
 * \returns uint8 socket handle 0 - 7
 * \returns 0xFF socket open error
 *
 * w5500_SocketOpen initializes and opens a socket on the specified port using
 * the socket mode specified by the flags.  Upon sucessful opening, the socket
 * handle will be returned for future use by the driver.  A return value of 
 * 0xFF indicates a socket open failure.
 *
 * Socket handle 0 is reserved for MACRAW operations, since socket 0 of the
 * W5500 is the only socket where MACRAW more operations are valid.
 *
 * \note the Tx_size and rx_size parameters are ignored and assumed to be 2K
 */
uint8 w5500_SocketOpen( uint16 port, uint8 flags)
{
	uint8 socket;
	int idx;
	
	/* default the socket to the error condition */
	socket = 0xFF;
	/* 
	 * If the MACRAW protocol was selected, make sure that socket 0 is
	 * available, otherwise flag an error and kick back to the user
	 */
	if ( (flags & W5500_PROTO_MACRAW) != 0) {
		if (w5500_ChipInfo.socketStatus[0] != W5500_SOCKET_AVAILALE) {
			return 0xFF;
		}
		else {
			socket = 0;
		}
	}
	else {
		/*
		 * scan the other sockets, starting at 1 (0 is ignore to support the
		 * MACRAW mode of operation) and look for the first available socket.
		 * once there is a socket available, exit the loop and store the socket
		 * id in the socket number.
		 */
		idx = 1;
		while ( (idx < W5500_MAX_SOCKETS) && (w5500_ChipInfo.socketStatus[idx] == W5500_SOCKET_OPEN) ) {
			++idx;
		}
		if ( idx >= W5500_MAX_SOCKETS ) {
			return 0xFF;
		}
		socket = idx;
	}
	/*
	 * Now that the socket is identified, declare it as open, then set the mode
	 * register and issue the socket open command
	 */
	w5500_ChipInfo.socketStatus[socket] = W5500_SOCKET_OPEN;
	w5500_Send( W5500_SREG_MR, W5500_SOCKET_BASE(socket), 1, &flags, 1);
	/*
	 * Initialize memory pointers for the W5500 device
	 */
	port = CYSWAP_ENDIAN16(port);
	w5500_Send( W5500_SREG_PORT, W5500_SOCKET_BASE(socket), 1, (uint8*)&port, 2);
	/*
	 * execute the open command, and check the result.  If the result
	 * was not 0, there was an error in the command, so set the socket
	 * as available, and return 0xFF error socket to safely exit the error
	 * condition.
	 */
	if (w5500_ExecuteSocketCommand(W5500_CR_OPEN, socket) != 0) {
		w5500_ChipInfo.socketStatus[socket] = W5500_SOCKET_AVAILALE;
		socket = 0xFF;
	}
	return socket;
}
/* ------------------------------------------------------------------------- */
/**
 * \brief Close an open socket
 * \param sock (uint8) the socket number to close
 *
 * _SocketClose closes an open socket connection optionally sending the
 * disconnection sequence.  After the command, the socket allocation is
 * cleared.
 *
 * \sa w500_SocketDisconnect
 */
uint8 w5500_SocketClose( uint8 sock, uint8 discon )
{
	uint8 status;
	uint8 ir;
	
	ir = 0xFF;
	
	/* Trap socket invalid handle errors */
	if (sock > 7) return CYRET_BAD_PARAM;
	/* Ignore close requests for sockets that are not open */
	if (w5500_ChipInfo.socketStatus[sock] == W5500_SOCKET_AVAILALE) return CYRET_BAD_PARAM;
	
	/*
	 * first read the status of the socket from the W5500, and return with an
	 * error code if the socket is already closed.
	 */
	w5500_Send(W5500_SREG_SR, W5500_SOCKET_BASE(sock),0,&status,1);
	if (status == W5500_SR_CLOSED) {
		w5500_ChipInfo.socketStatus[sock] = W5500_SOCKET_AVAILALE;
		return CYRET_CANCELED;
	}
	
	/*
	 * the socket was allocated and also in a state that is not already closed,
	 * so issue the close command to the device to terminate the connection
	 */
	if (discon != 0) {
		status = w5500_ExecuteSocketCommand( sock, W5500_CR_DISCON );
	}
	status = w5500_ExecuteSocketCommand( sock, W5500_CR_CLOSE );
	
	if (status == CYRET_SUCCESS ) {
		w5500_ChipInfo.socketStatus[sock] = W5500_SOCKET_AVAILALE;
	}
	/*
	 * clear pending socket interrupts
	 */
	w5500_Send(W5500_SREG_IR, W5500_SOCKET_BASE(sock), 1, &ir, 1 );
	
	return status;
}
/* ------------------------------------------------------------------------ */
/**
 * \brief Close a socket and send the termination sequence to an open connection
 * \param sock (uint8) The socket number to colose
 * \returns 0 successful closure of the socket
 * \returns 0xFF socket was already closed, or not allocated
 * \retutns uint8 value indicating faulure of the discon command execution
 */
uint8 w5500_SocketDisconnect( uint8 sock )
{
	return w5500_SocketClose( sock, 1);
}
/* ------------------------------------------------------------------------ */
/**
 * \brief Check to see if a SEND operation was completed.
 */
uint8 w5500_SocketSendComplete( uint8 socket )
{
	uint8 ir;
	uint8 result;
	
	if (W5500_SOCKET_BAD( socket ) ) return CYRET_BAD_PARAM;

	/* Set the result to waiting to process */
	result = CYRET_STARTED;
	/* Read the Socket IR and check the flags for a send OK */
	w5500_Send(W5500_SREG_IR, W5500_SOCKET_BASE(socket),0,&ir,1);
	if ( (ir&W5500_IR_SEND_OK) == 0 ) {
		/*
		 * The Send was not complete, so now read the status to determine if the
		 * socket was closed remotely.
		 */
		w5500_Send(W5500_SREG_SR, W5500_SOCKET_BASE(socket),0,&ir,1);
		if ( ir == W5500_SR_CLOSED ) {
			w5500_SocketClose( socket, 1 );
			result = CYRET_CANCELED;
		}
	}
	else {
		result = CYRET_FINISHED;
	}
	
	return result;
}
/* [] END OF FILE */

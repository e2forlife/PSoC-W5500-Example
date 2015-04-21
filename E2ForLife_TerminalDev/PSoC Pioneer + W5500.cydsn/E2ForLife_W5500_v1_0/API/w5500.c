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
 * \file w5500.c
 * \author Chuck Erhardt (chuck@e2forlife.com)
 * 
 * This file contains the main driver code for low-level device protocol
 * access as well as driver initialization, device initialization and startup
 * code for the WizNET iW5500 device.
 */
/* ======================================================================== */
#include <cylib.h>
#include <cytypes.h>
#include <string.h>

#include <`$CS_INSTANCE`.h>
#include <`$SPI_INSTANCE`.h>

#if defined (CY_SCB_`$SPI_INSTANCE`_H)
	#include <`$SPI_INSTANCE`_SPI_UART.h>
#endif

#include "`$INSTANCE_NAME`.h"


///**
// * constant conversion lookup table to convert socket number
// * to register block bank in the W5500 device.
// */
//const uint8 `$INSTANCE_NAME`_socket_reg[8] = {
//	`$INSTANCE_NAME`_BLOCK_S0_REG,
//	`$INSTANCE_NAME`_BLOCK_S1_REG,
//	`$INSTANCE_NAME`_BLOCK_S2_REG,
//	`$INSTANCE_NAME`_BLOCK_S3_REG,
//	`$INSTANCE_NAME`_BLOCK_S4_REG,
//	`$INSTANCE_NAME`_BLOCK_S5_REG,
//	`$INSTANCE_NAME`_BLOCK_S6_REG,
//	`$INSTANCE_NAME`_BLOCK_S7_REG
//};
///**
// * constant conversion lookup table to convert socket number
// * to TX Memory bank in the W5500 device.
// */
//const uint8 `$INSTANCE_NAME`_socket_tx[8] = {
//	`$INSTANCE_NAME`_BLOCK_S0_TXB,
//	`$INSTANCE_NAME`_BLOCK_S1_TXB,
//	`$INSTANCE_NAME`_BLOCK_S2_TXB,
//	`$INSTANCE_NAME`_BLOCK_S3_TXB,
//	`$INSTANCE_NAME`_BLOCK_S4_TXB,
//	`$INSTANCE_NAME`_BLOCK_S5_TXB,
//	`$INSTANCE_NAME`_BLOCK_S6_TXB,
//	`$INSTANCE_NAME`_BLOCK_S7_TXB
//};
///**
// * constant conversion lookup table to convert socket number
// * to register block bank in the W5500 device.
// */
//const uint8 `$INSTANCE_NAME`_socket_rx[8] = {
//	`$INSTANCE_NAME`_BLOCK_S0_RXB,
//	`$INSTANCE_NAME`_BLOCK_S1_RXB,
//	`$INSTANCE_NAME`_BLOCK_S2_RXB,
//	`$INSTANCE_NAME`_BLOCK_S3_RXB,
//	`$INSTANCE_NAME`_BLOCK_S4_RXB,
//	`$INSTANCE_NAME`_BLOCK_S5_RXB,
//	`$INSTANCE_NAME`_BLOCK_S6_RXB,
//	`$INSTANCE_NAME`_BLOCK_S7_RXB
//};
/* ------------------------------------------------------------------------ */

#define `$INSTANCE_NAME`_CS_MASK               ( 1<<0 )
#define `$INSTANCE_NAME`_RESET_DELAY           ( 100 )

uint8 `$INSTANCE_NAME`_socketStatus[`$INSTANCE_NAME`_MAX_SOCKETS];

/* ------------------------------------------------------------------------ */
/**
 * \brief W5500 interface protocol write
 * \param offset address offset for the buffer access
 * \param block_select register block of the W5500 to access
 * \param buffer pointer to the data to write
 * \param len length of the data to send
 */
void `$INSTANCE_NAME`_Send(uint16 offset, uint8 block_select, uint8 write, uint8 *buffer, uint16 len)
#if !defined(CY_SCB_`$SPI_INSTANCE`_H)
{
	//uint8 status;
	int count;
	
	/* wait for SPI operations to complete */
	do {
		status = `$SPI_INSTANCE`_ReadTxStatus() & (`$SPI_INSTANCE`_STS_SPI_IDLE|`$SPI_INSTANCE`_STS_SPI_DONE);
	} while ( status == 0);
	
	/* set write bit in the control phase data */
	block_select = block_select | ((write!=0)?0x04:0); 
	/* select the data mode based on the block length */
	if (len == 1) {
		block_select = block_select | 0x01;
	}
	else if (len == 2) {
		block_select = block_select | 0x02;
	}
	else if (len == 4) {
		block_select = block_select | 0x03;
	}
	
	/* select the device */
	`$CS_INSTANCE`_Write(~(`$INSTANCE_NAME`_CS_MASK));
	/* send the address phase data */
	`$SPI_INSTANCE`_WriteTxData( HI8(offset) );
	`$SPI_INSTANCE`_WriteTxData( LO8(offset) );
	/* send the control phase data */
	`$SPI_INSTANCE`_WriteTxData( block_select );
	/*
	 * clear data read during the previous SPI write.  FIrst, wait for data
	 * to arrive in the RX fifo (if not has yet been received), then, read the
	 * data and wait for 3 data elements to be read (the length of the header
	 * sent during the address and control phase of the protocol
	 */
	count = 3;
	while ( (count != 0) || (`$SPI_INSTANCE`_GetRxBufferSize() ) ) {
		if (`$SPI_INSTANCE`_GetRxBufferSize() ) {
			`$SPI_INSTANCE`_ReadRxData();
			count = (count==0)?0:count-1;
		}
		CyDelayUs(5);
	}
	/* 
	 * Now that the Receive FIFO has been flushed, send data through
	 * the SPI port and wait for the receive buffer to contain data. Once the
	 * receive buffer contains data, read the data and store it in to the
	 * buffer.
	 */
	count = 0;
	while ( count < len) {
		
		if (`$SPI_INSTANCE`_GetTxBufferSize() < 4) {
			`$SPI_INSTANCE`_WriteTxData( (write==1)?buffer[count] : 0xFF );
			while (`$SPI_INSTANCE`_GetRxBufferSize() == 0);
			if (write == 0) {
				buffer[count] = `$SPI_INSTANCE`_ReadRxData();
			}
			else {
				`$SPI_INSTANCE`_ReadRxData();
			}
			++count;
		}
		CyDelayUs(5);
	}
	/* Turn off the chip select. */
	do {
		status = `$SPI_INSTANCE`_ReadTxStatus() & (`$SPI_INSTANCE`_STS_SPI_IDLE|`$SPI_INSTANCE`_STS_SPI_DONE);
	} while ( status == 0);
	`$CS_INSTANCE`_Write(0xFF);
	`$SPI_INSTANCE`_ClearFIFO();
}
#else
	/*
	 * `$INSTANCE_NAME`_Send (SCB Mode)
	 */
{
	int count;
//	uint8 status;
	
	/* wait for SPI operations to complete */
	while( `$SPI_INSTANCE`_SpiIsBusBusy() != 0) {
		CyDelay(1);
	}
		
	/* set write bit in the control phase data */
	block_select = block_select | ((write!=0)?0x04:0); 
	/* select the data mode based on the block length */
	if (len == 1) {
		block_select = block_select | 0x01;
	}
	else if (len == 2) {
		block_select = block_select | 0x02;
	}
	else if (len == 4) {
		block_select = block_select | 0x03;
	}
	
	/* select the device */
	`$CS_INSTANCE`_Write(~(`$INSTANCE_NAME`_CS_MASK));
	/* send the address phase data */
	`$SPI_INSTANCE`_SpiUartWriteTxData( HI8(offset) );
	`$SPI_INSTANCE`_SpiUartWriteTxData( LO8(offset) );
	/* send the control phase data */
	`$SPI_INSTANCE`_SpiUartWriteTxData( block_select );
	/*
	 * clear data read during the previous SPI write.  FIrst, wait for data
	 * to arrive in the RX fifo (if not has yet been received), then, read the
	 * data and wait for 3 data elements to be read (the length of the header
	 * sent during the address and control phase of the protocol
	 */
	count = 3;
	while ( (count != 0) || (`$SPI_INSTANCE`_SpiUartGetRxBufferSize() ) ) {
		if (`$SPI_INSTANCE`_SpiUartGetRxBufferSize() ) {
			`$SPI_INSTANCE`_SpiUartReadRxData();
			count = (count==0)?0:count-1;
		}
		CyDelayUs(5);
	}
	/* 
	 * Now that the Receive FIFO has been flushed, send data through
	 * the SPI port and wait for the receive buffer to contain data. Once the
	 * receive buffer contains data, read the data and store it in to the
	 * buffer.
	 */
	count = 0;
	while ( count < len) {
		
		if (`$SPI_INSTANCE`_SpiUartGetTxBufferSize() < 4) {
			`$SPI_INSTANCE`_SpiUartWriteTxData( (write==1)?buffer[count] : 0xFF );
			while (`$SPI_INSTANCE`_SpiUartGetRxBufferSize() == 0);
			if (write == 0) {
				buffer[count] = `$SPI_INSTANCE`_SpiUartReadRxData();
			}
			else {
				`$SPI_INSTANCE`_SpiUartReadRxData();
			}
			++count;
		}
		CyDelayUs(5);
	}
	
	/* Turn off the chip select. */
	while (`$SPI_INSTANCE`_SpiIsBusBusy() != 0) {
		CyDelay(1);
	}
	`$CS_INSTANCE`_Write(0xFF);
	`$SPI_INSTANCE`_SpiUartClearRxBuffer();
	`$SPI_INSTANCE`_SpiUartClearTxBuffer();
}
#endif
/* ------------------------------------------------------------------------ */
uint16 `$INSTANCE_NAME`_RxDataReady( uint8 socket )
{
	uint16 first, second;
	
	/* quit on invalid sockets */
	if (`$INSTANCE_NAME`_SOCKET_BAD(socket)) return 0;
		
	first = 0;
	second = 0;
	do {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_RX_RSR, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,(uint8*)&first,2);
		if (first != 0) {
			`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_RX_RSR, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,(uint8*)&second,2);
		}
	}
	while (first != second );
	
	return CYSWAP_ENDIAN16(second);
}
/* ------------------------------------------------------------------------ */
uint16 `$INSTANCE_NAME`_TxBufferFree( uint8 socket )
{
	uint16 first, second;
	
	/* quit on invalid sockets */
	if (`$INSTANCE_NAME`_SOCKET_BAD(socket)) return 0;
	
	first = 0;
	second = 0;
	do {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_TX_FSR, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,(uint8*)&first,2);
		if (first != 0) {
			`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_TX_FSR, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,(uint8*)&second,2);
		}
	}
	while (first != second );
	
	return CYSWAP_ENDIAN16(second);
}
/* ------------------------------------------------------------------------ */
void `$INSTANCE_NAME`_Reset( void )
{
	uint8 status;
	
	/*
	 * issue a mode register reset to the W5500 in order to set default
	 * register contents for the chip.
	 */
	status = 0x80;
	`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_MODE,`$INSTANCE_NAME`_BLOCK_COMMON,1, &status, 1);
	/*
	 * Wait for the mode register to clear the reset bit, thus indicating
	 * that the reset command has been completed.
	 */
	do {
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_REG_MODE, `$INSTANCE_NAME`_BLOCK_COMMON, 0, &status, 1);
	}
	while ( (status & 0x80) != 0 );
}
/* ------------------------------------------------------------------------ */
cystatus `$INSTANCE_NAME`_Init( uint8* gateway, uint8* subnet, uint8* mac, uint8 *ip )
{
	int socket;
	uint8 socket_cfg[14] = { `$INSTANCE_NAME`_SOCKET_MEM, `$INSTANCE_NAME`_SOCKET_MEM, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8 config[18];
		
	/*
	 * build chip initialization from the default strings set in the
	 * configuration dialog
	 */
	for (socket = 0; socket < 8; ++socket) {
		`$INSTANCE_NAME`_socketStatus[socket] = `$INSTANCE_NAME`_SOCKET_AVAILALE;
		/* Send socket register setup to the chip */
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_SREG_RXBUF_SIZE, `$INSTANCE_NAME`_SOCKET_BASE(socket), 1, &socket_cfg[0], 14);
	}
	
	if ( gateway != NULL) {
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_REG_GAR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, gateway, 4);
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_REG_GAR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 4 );
		for(socket=0;(socket<4)&&(gateway[socket]==config[socket]);++socket);
		if (socket < 4) return CYRET_BAD_DATA;
	}
	
	if (subnet != NULL) {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SUBR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, subnet, 4);
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SUBR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 4);
		for(socket=0;(socket<4)&&(subnet[socket]==config[socket]);++socket);
		if (socket < 4) return CYRET_BAD_DATA;
	}		

	if (mac != NULL) {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SHAR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, mac, 6);
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SHAR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 6);
		for(socket=0;(socket<6)&&(mac[socket]==config[socket]);++socket);
		if (socket < 6) return CYRET_BAD_DATA;
	}		

	if (ip != NULL) {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SIPR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, ip, 4);
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SIPR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 4);
		for(socket=0;(socket<4)&&(ip[socket]==config[socket]);++socket);
		if (socket < 4) return CYRET_BAD_DATA;
	}		
	
	return CYRET_SUCCESS;
}
/* ------------------------------------------------------------------------ */
/**
 * \brief Initialize the W5500 with the default settings configured in Creator
 * 
 * This function calls the StartEx function to configure the W550 device using
 * the default setting as setup by the user in the configuration dialog in the
 * Creator tools.
 */
cystatus `$INSTANCE_NAME`_Start( void )
{	
	cystatus result;
	
	result = `$INSTANCE_NAME`_StartEx("`$GATEWAY`","`$SUBNET`","`$MAC`","`$IP`");
	return result;	
}
/* ------------------------------------------------------------------------ */
/**
 * \brief configure the W5500 with user settings
 * \param config the Configuration for the W5500 device
 * 
 * This function converts the configuration in to the for useable by the W5500
 * and builds a burst data buffer to transfer to the W5500.  the configuration
 * paramters are stored in the _ChipInfo struct for later use by the driver
 * with no conversions required.
 */
cystatus `$INSTANCE_NAME`_StartEx( const char *gateway, const char *subnet, const char *mac, const char *ip )
{
	uint8 result;
	uint32 timeout;
	uint32 gar, subr, sipr;
	uint8 shar[6];
	uint8 *g, *s, *m, *i;
	
	/*
	 * Wait for initial power-on PLL Lock, and issue a device reset
	 * to initialize the registers
	 */
	CyDelay(`$INSTANCE_NAME`_RESET_DELAY);
	
	/*
	 * use IoT utlity functions to convert from ASCII data to binary data
	 * used by the driver, and store it in the ChipInfo for later use.
	 */
	if ((strlen(gateway) > 0 )&&(gateway!=NULL)) {
		gar = `$INSTANCE_NAME`_ParseIP(gateway);
		g = (uint8*) &gar;
	}
	else {
		g = NULL;
	}
	
	if( (strlen(subnet)>0) &&( subnet !=NULL)) {
		subr = `$INSTANCE_NAME`_ParseIP(subnet);
		s = (uint8*) &subr;
	} 
	else {
		s = NULL;
	}
	
	if ((strlen(mac)>0) && (mac !=NULL) ) {
		`$INSTANCE_NAME`_ParseMAC(mac, &shar[0] );
		m = shar;
	}
	else {
		m = NULL;
	}
	
	if ((strlen(ip)>0) && (ip !=NULL) ) {
		sipr = `$INSTANCE_NAME`_ParseIP(ip);
		i = (uint8*)&sipr;
	}
	else {
		i = NULL;
	}
	 
	/*
	 * Attempt to initialize the device for approx. 1 second. If after
	 * 1 second there is still a verification failure, return a timeout
	 * error, otherwise return success!
	 */
	do {	
		`$INSTANCE_NAME`_Reset();
		result = `$INSTANCE_NAME`_Init(g,s,m,i);
		if (result != CYRET_SUCCESS) {
			CyDelay(1);
			++timeout;
			result = CYRET_TIMEOUT;
		}
	}
	while ( (result != CYRET_SUCCESS) && (timeout < 1000) );
	
	return result;
}
/* ------------------------------------------------------------------------ */
void `$INSTANCE_NAME`_GetMac(uint8* mac)
{
	`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SHAR,`$INSTANCE_NAME`_BLOCK_COMMON,0,mac,6);
}
/* ------------------------------------------------------------------------ */
uint32 `$INSTANCE_NAME`_GetIp( void )
{
	uint32 ipr;
	`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SIPR, `$INSTANCE_NAME`_BLOCK_COMMON,0,(uint8*)&ipr,4);
	return ipr;
}
/* ------------------------------------------------------------------------ */
uint16 `$INSTANCE_NAME`_GetTxLength(uint8 socket, uint16 len, uint8 flags)
{
	uint16 tx_length;
	uint16 max_packet;
	uint8 buf_size;
	
	if (`$INSTANCE_NAME`_SOCKET_BAD(socket) ) return 0;
	
	tx_length = `$INSTANCE_NAME`_TxBufferFree( socket );
	if ( (tx_length < len ) && ((flags&`$INSTANCE_NAME`_TXRX_FLG_WAIT) != 0) ) {
		/* 
		 * there is not enough room in the buffer, but the caller requested
		 * this to block until there was free space. So, check the memory
		 * size to determine if the tx buffer is big enough to handle the
		 * data block without fragmentation.
		 */
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_TXBUF_SIZE, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,&buf_size,1);
		max_packet = (buf_size == 0)? 0 : (0x400 << (buf_size-1));
		/*
		 * now that we know the max buffer size, if it is smaller than the
		 * requested transmit lenght, we have an error, so return 0
		 */
		if (max_packet < len ) return 0;
		/* otherwise, we will wait for the room in the buffer */
		do {
			tx_length = `$INSTANCE_NAME`_TxBufferFree( socket );
		}
		while ( tx_length < len );
	}
	else {
		tx_length = len;
	}
	
	return tx_length;
}
/* ------------------------------------------------------------------------ */
cystatus `$INSTANCE_NAME`_WriteTxData(uint8 socket, uint8 *buffer, uint16 tx_length, uint8 flags)
{
	uint16 ptr;
	cystatus result;
	
		/*
	 * The length of the Tx data block has now been determined, and can be
	 * copied in to the W5500 buffer memory. First read the pointer, then
	 * write data from the pointer forward, lastly update the pointer and issue
	 * the SEND command.
	 */
	`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_SREG_TX_WR, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,(uint8*)&ptr,2);
	ptr = CYSWAP_ENDIAN16( ptr );
	`$INSTANCE_NAME`_Send( ptr, `$INSTANCE_NAME`_TX_BASE(socket),1,buffer,tx_length);
	ptr += tx_length;
	ptr = CYSWAP_ENDIAN16( ptr );
	`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_TX_WR, `$INSTANCE_NAME`_SOCKET_BASE(socket),1,(uint8*)&ptr,2);
	
	`$INSTANCE_NAME`_ExecuteSocketCommand( socket, `$INSTANCE_NAME`_CR_SEND );
	result = `$INSTANCE_NAME`_SocketSendComplete(socket);
	
	if ( (flags & `$INSTANCE_NAME`_TXRX_FLG_WAIT) != 0) {
		/*
		 * block until send is complete
		 */
		do {
			CyDelay(1);
			result = `$INSTANCE_NAME`_SocketSendComplete(socket);
		}
		while( (result != CYRET_FINISHED) && (result != CYRET_CANCELED) );
	}
	
	return result;
}
/* ======================================================================== */
/** @} */
/* [] END OF FILE */

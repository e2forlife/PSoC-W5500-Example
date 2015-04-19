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

#include <ETH_CSN.h>
#include <SPI0.h>

#if defined (CY_SCB_SPI0_H)
	#include <SPI0_SPI_UART.h>
#endif

#include "w5500.h"
#include "iot.h"


///**
// * constant conversion lookup table to convert socket number
// * to register block bank in the W5500 device.
// */
//const uint8 w5500_socket_reg[8] = {
//	W5500_BLOCK_S0_REG,
//	W5500_BLOCK_S1_REG,
//	W5500_BLOCK_S2_REG,
//	W5500_BLOCK_S3_REG,
//	W5500_BLOCK_S4_REG,
//	W5500_BLOCK_S5_REG,
//	W5500_BLOCK_S6_REG,
//	W5500_BLOCK_S7_REG
//};
///**
// * constant conversion lookup table to convert socket number
// * to TX Memory bank in the W5500 device.
// */
//const uint8 w5500_socket_tx[8] = {
//	W5500_BLOCK_S0_TXB,
//	W5500_BLOCK_S1_TXB,
//	W5500_BLOCK_S2_TXB,
//	W5500_BLOCK_S3_TXB,
//	W5500_BLOCK_S4_TXB,
//	W5500_BLOCK_S5_TXB,
//	W5500_BLOCK_S6_TXB,
//	W5500_BLOCK_S7_TXB
//};
///**
// * constant conversion lookup table to convert socket number
// * to register block bank in the W5500 device.
// */
//const uint8 w5500_socket_rx[8] = {
//	W5500_BLOCK_S0_RXB,
//	W5500_BLOCK_S1_RXB,
//	W5500_BLOCK_S2_RXB,
//	W5500_BLOCK_S3_RXB,
//	W5500_BLOCK_S4_RXB,
//	W5500_BLOCK_S5_RXB,
//	W5500_BLOCK_S6_RXB,
//	W5500_BLOCK_S7_RXB
//};
/* ------------------------------------------------------------------------ */

#define W5500_CS_MASK               ( 1<<0 )
#define W5500_RESET_DELAY           ( 100 )

uint8 W5500_socketStatus[W5500_MAX_SOCKETS];

/* ------------------------------------------------------------------------ */
/**
 * \brief W5500 interface protocol write
 * \param offset address offset for the buffer access
 * \param block_select register block of the W5500 to access
 * \param buffer pointer to the data to write
 * \param len length of the data to send
 */
void w5500_Send(uint16 offset, uint8 block_select, uint8 write, uint8 *buffer, uint16 len)
#if !defined(CY_SCB_SPI0_H)
{
	//uint8 status;
	int count;
	
	/* wait for SPI operations to complete */
	do {
		status = SPI0_ReadTxStatus() & (SPI0_STS_SPI_IDLE|SPI0_STS_SPI_DONE);
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
	ETH_CSN_Write(~(W5500_CS_MASK));
	/* send the address phase data */
	SPI0_WriteTxData( HI8(offset) );
	SPI0_WriteTxData( LO8(offset) );
	/* send the control phase data */
	SPI0_WriteTxData( block_select );
	/*
	 * clear data read during the previous SPI write.  FIrst, wait for data
	 * to arrive in the RX fifo (if not has yet been received), then, read the
	 * data and wait for 3 data elements to be read (the length of the header
	 * sent during the address and control phase of the protocol
	 */
	count = 3;
	while ( (count != 0) || (SPI0_GetRxBufferSize() ) ) {
		if (SPI0_GetRxBufferSize() ) {
			SPI0_ReadRxData();
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
		
		if (SPI0_GetTxBufferSize() < 4) {
			SPI0_WriteTxData( (write==1)?buffer[count] : 0xFF );
			while (SPI0_GetRxBufferSize() == 0);
			if (write == 0) {
				buffer[count] = SPI0_ReadRxData();
			}
			else {
				SPI0_ReadRxData();
			}
			++count;
		}
		CyDelayUs(5);
	}
	/* Turn off the chip select. */
	do {
		status = SPI0_ReadTxStatus() & (SPI0_STS_SPI_IDLE|SPI0_STS_SPI_DONE);
	} while ( status == 0);
	ETH_CSN_Write(0xFF);
	SPI0_ClearFIFO();
}
#else
	/*
	 * W5500_Send (SCB Mode)
	 */
{
	int count;
//	uint8 status;
	
	/* wait for SPI operations to complete */
	while( SPI0_SpiIsBusBusy() != 0) {
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
	ETH_CSN_Write(~(W5500_CS_MASK));
	/* send the address phase data */
	SPI0_SpiUartWriteTxData( HI8(offset) );
	SPI0_SpiUartWriteTxData( LO8(offset) );
	/* send the control phase data */
	SPI0_SpiUartWriteTxData( block_select );
	/*
	 * clear data read during the previous SPI write.  FIrst, wait for data
	 * to arrive in the RX fifo (if not has yet been received), then, read the
	 * data and wait for 3 data elements to be read (the length of the header
	 * sent during the address and control phase of the protocol
	 */
	count = 3;
	while ( (count != 0) || (SPI0_SpiUartGetRxBufferSize() ) ) {
		if (SPI0_SpiUartGetRxBufferSize() ) {
			SPI0_SpiUartReadRxData();
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
		
		if (SPI0_SpiUartGetTxBufferSize() < 4) {
			SPI0_SpiUartWriteTxData( (write==1)?buffer[count] : 0xFF );
			while (SPI0_SpiUartGetRxBufferSize() == 0);
			if (write == 0) {
				buffer[count] = SPI0_SpiUartReadRxData();
			}
			else {
				SPI0_SpiUartReadRxData();
			}
			++count;
		}
		CyDelayUs(5);
	}
	
	/* Turn off the chip select. */
	while (SPI0_SpiIsBusBusy() != 0) {
		CyDelay(1);
	}
	ETH_CSN_Write(0xFF);
	SPI0_SpiUartClearRxBuffer();
	SPI0_SpiUartClearTxBuffer();
}
#endif
/* ------------------------------------------------------------------------ */
uint16 w5500_RxDataReady( uint8 socket )
{
	uint16 first, second;
	
	/* quit on invalid sockets */
	if (W5500_SOCKET_BAD(socket)) return 0;
		
	first = 0;
	second = 0;
	do {
		w5500_Send(W5500_SREG_RX_RSR, W5500_SOCKET_BASE(socket),0,(uint8*)&first,2);
		if (first != 0) {
			w5500_Send(W5500_SREG_RX_RSR, W5500_SOCKET_BASE(socket),0,(uint8*)&second,2);
		}
	}
	while (first != second );
	
	return CYSWAP_ENDIAN16(second);
}
/* ------------------------------------------------------------------------ */
uint16 w5500_TxBufferFree( uint8 socket )
{
	uint16 first, second;
	
	/* quit on invalid sockets */
	if (W5500_SOCKET_BAD(socket)) return 0;
	
	first = 0;
	second = 0;
	do {
		w5500_Send(W5500_SREG_TX_FSR, W5500_SOCKET_BASE(socket),0,(uint8*)&first,2);
		if (first != 0) {
			w5500_Send(W5500_SREG_TX_FSR, W5500_SOCKET_BASE(socket),0,(uint8*)&second,2);
		}
	}
	while (first != second );
	
	return CYSWAP_ENDIAN16(second);
}
/* ------------------------------------------------------------------------ */
void w5500_Reset( void )
{
	uint8 status;
	
	/*
	 * issue a mode register reset to the W5500 in order to set default
	 * register contents for the chip.
	 */
	status = 0x80;
	w5500_Send(W5500_REG_MODE,W5500_BLOCK_COMMON,1, &status, 1);
	/*
	 * Wait for the mode register to clear the reset bit, thus indicating
	 * that the reset command has been completed.
	 */
	do {
		w5500_Send( W5500_REG_MODE, W5500_BLOCK_COMMON, 0, &status, 1);
	}
	while ( (status & 0x80) != 0 );
}
/* ------------------------------------------------------------------------ */
cystatus w5500_Init( uint8* gateway, uint8* subnet, uint8* mac, uint8 *ip )
{
	int socket;
	uint8 socket_cfg[14] = { W5500_SOCKET_MEM, W5500_SOCKET_MEM, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8 config[18];
		
	/*
	 * build chip initialization from the default strings set in the
	 * configuration dialog
	 */
	for (socket = 0; socket < 8; ++socket) {
		W5500_socketStatus[socket] = W5500_SOCKET_AVAILALE;
		/* Send socket register setup to the chip */
		w5500_Send( W5500_SREG_RXBUF_SIZE, W5500_SOCKET_BASE(socket), 1, &socket_cfg[0], 14);
	}
	
	if ( gateway != NULL) {
		w5500_Send( W5500_REG_GAR, W5500_BLOCK_COMMON, 1, gateway, 4);
		w5500_Send( W5500_REG_GAR, W5500_BLOCK_COMMON, 0, config, 4 );
		for(socket=0;(socket<4)&&(gateway[socket]==config[socket]);++socket);
		if (socket < 4) return CYRET_BAD_DATA;
	}
	
	if (subnet != NULL) {
		w5500_Send(W5500_REG_SUBR, W5500_BLOCK_COMMON, 1, subnet, 4);
		w5500_Send(W5500_REG_SUBR, W5500_BLOCK_COMMON, 0, config, 4);
		for(socket=0;(socket<4)&&(subnet[socket]==config[socket]);++socket);
		if (socket < 4) return CYRET_BAD_DATA;
	}		

	if (mac != NULL) {
		w5500_Send(W5500_REG_SHAR, W5500_BLOCK_COMMON, 1, mac, 6);
		w5500_Send(W5500_REG_SHAR, W5500_BLOCK_COMMON, 0, config, 6);
		for(socket=0;(socket<6)&&(mac[socket]==config[socket]);++socket);
		if (socket < 6) return CYRET_BAD_DATA;
	}		

	if (ip != NULL) {
		w5500_Send(W5500_REG_SIPR, W5500_BLOCK_COMMON, 1, ip, 4);
		w5500_Send(W5500_REG_SIPR, W5500_BLOCK_COMMON, 0, config, 4);
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
cystatus w5500_Start( void )
{	
	uint8 result;
	uint32 timeout;

	/*
	 * Wait for initial power-on PLL Lock, and issue a device reset
	 * to initialize the registers
	 */
	CyDelay(W5500_RESET_DELAY);
		 
	/*
	 * Attempt to initialize the device for approx. 1 second. If after
	 * 1 second there is still a verification failure, return a timeout
	 * error, otherwise return success!
	 */
	do {	
		w5500_Reset();
		result = w5500_Init(NULL, NULL, NULL, NULL);
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
/**
 * \brief configure the W5500 with user settings
 * \param config the Configuration for the W5500 device
 * 
 * This function converts the configuration in to the for useable by the W5500
 * and builds a burst data buffer to transfer to the W5500.  the configuration
 * paramters are stored in the _ChipInfo struct for later use by the driver
 * with no conversions required.
 */
cystatus w5500_StartEx( const char *gateway, const char *subnet, const char *mac, const char *ip )
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
	CyDelay(W5500_RESET_DELAY);
	
	/*
	 * use IoT utlity functions to convert from ASCII data to binary data
	 * used by the driver, and store it in the ChipInfo for later use.
	 */
	if ((strlen(gateway) > 0 )&&(gateway!=NULL)) {
		gar = IOT_ParseIP(gateway);
		g = (uint8*) &gar;
	}
	else {
		g = NULL;
	}
	
	if( (strlen(subnet)>0) &&( subnet !=NULL)) {
		subr = IOT_ParseIP(subnet);
		s = (uint8*) &subr;
	} 
	else {
		s = NULL;
	}
	
	if ((strlen(mac)>0) && (mac !=NULL) ) {
		IOT_ParseMAC(mac, &shar[0] );
		m = shar;
	}
	else {
		m = NULL;
	}
	
	if ((strlen(ip)>0) && (ip !=NULL) ) {
		sipr = IOT_ParseIP(ip);
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
		w5500_Reset();
		result = w5500_Init(g,s,m,i);
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
/** @} */
/* [] END OF FILE */

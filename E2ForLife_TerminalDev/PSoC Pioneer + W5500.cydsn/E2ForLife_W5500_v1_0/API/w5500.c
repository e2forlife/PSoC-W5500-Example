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

#include <CyLib.h>
#include <cytypes.h>
#include <string.h>

#include <`$CS_INSTANCE`.h>
#include <`$SPI_INSTANCE`.h>

#if defined (CY_SCB_`$SPI_INSTANCE`_H)
	#include <`$SPI_INSTANCE`_SPI_UART.h>
#endif

#include "`$INSTANCE_NAME`.h"
#include "`$INSTANCE_NAME`_util.h"

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

#define `$INSTANCE_NAME`_CS_MASK        ( 1 << 0 )
#define `$INSTANCE_NAME`_CS_ASSERT      ( 0 )
#define `$INSTANCE_NAME`_CS_DEASSERT    ( 1 )
#define `$INSTANCE_NAME`_RESET_DELAY    ( 100 )

uint8_t `$INSTANCE_NAME`_socketStatus[`$INSTANCE_NAME`_MAX_SOCKETS];

/**
 * \brief W5500 interface protocol write
 * \param offset address offset for the buffer access
 * \param block_select register block of the W5500 to access
 * \param buffer pointer to the data to write
 * \param len length of the data to send
 */
void `$INSTANCE_NAME`_Send( uint16_t offset, uint8_t block_select, uint8_t write,
                            uint8_t *buffer, uint16_t len )
{
#if !defined(CY_SCB_`$SPI_INSTANCE`_H)
	uint8_t status = 0;
	int count = 0;
	
	// wait for SPI operations to complete
	do {
		status = `$SPI_INSTANCE`_ReadTxStatus() & (`$SPI_INSTANCE`_STS_SPI_IDLE | `$SPI_INSTANCE`_STS_SPI_DONE);
	} while ( status == 0 );
	
	// set write bit in the control phase data
	block_select = block_select | (( write != 0) ? 0x04 : 0); 
    
	// select the data mode based on the block length
	if ( 1 == len ) {
		block_select |= 0x01;
	} else if ( 2 == len ) {
		block_select |= 0x02;
	} else if ( 4 == len ) {
		block_select |= 0x03;
	}
	
	`$CS_INSTANCE`_Write(~(`$INSTANCE_NAME`_CS_MASK));
    
	// send the address phase data
	`$SPI_INSTANCE`_WriteTxData( HI8(offset) );
	`$SPI_INSTANCE`_WriteTxData( LO8(offset) );
    
	// send the control phase data
	`$SPI_INSTANCE`_WriteTxData( block_select );
    
	// clear data read during the previous SPI write.  FIrst, wait for data
	// to arrive in the RX fifo (if not has yet been received), then, read the
	// data and wait for 3 data elements to be read (the length of the header
	// sent during the address and control phase of the protocol
#if 1
	count = 3;
	while ( (count != 0) || (`$SPI_INSTANCE`_GetRxBufferSize() ) ) {
		if (`$SPI_INSTANCE`_GetRxBufferSize() ) {
			`$SPI_INSTANCE`_ReadRxData();
			count = (count == 0) ? 0 : count - 1;
		}
		CyDelayUs(5);
	}
#else
    //while ( `$SPI_INSTANCE`_GetRxBufferSize() ) {
    //    `$SPI_INSTANCE`_ReadRxData();
    //    CyDelayUs(5);
    //}
    
    `$SPI_INSTANCE`_ClearRxBuffer();
    `$SPI_INSTANCE`_ClearTxBuffer();
    
#endif
	 
	// Now that the Receive FIFO has been flushed, send data through
	// the SPI port and wait for the receive buffer to contain data. Once the
	// receive buffer contains data, read the data and store it in to the
	// buffer.
#if 0
    count = 0;
    
    #if 1
    for ( count = 0; count < len; count++ ) {
        `$SPI_INSTANCE`_WriteTxData( (write == 1)? buffer[count] : 0xFF );
    }
    
    for ( count = 0; count < len; count++ ) {
        if ( 0 == write ) {
            buffer[count] = `$SPI_INSTANCE`_ReadRxData();
        } else {
            `$SPI_INSTANCE`_ReadRxData();
        }
    }
    
    #else
    while ( count < len ) {
        
        `$SPI_INSTANCE`_WriteTxData( (write == 1)? buffer[count] : 0xFF );
        
        if ( 0 == write ) {
            buffer[count] = `$SPI_INSTANCE`_ReadRxData();
        } else {
            `$SPI_INSTANCE`_ReadRxData();
        }
        ++count;
        
        CyDelayUs(5);
        
    }
    #endif
    
#else
	count = 0;
	while ( count < len ) {
		if (`$SPI_INSTANCE`_GetTxBufferSize() < 4) {
			`$SPI_INSTANCE`_WriteTxData( (write==1)?buffer[count] : 0xFF );
			while (`$SPI_INSTANCE`_GetRxBufferSize() == 0);
			if (write == 0) {
				buffer[count] = `$SPI_INSTANCE`_ReadRxData();
			} else {
				`$SPI_INSTANCE`_ReadRxData();
			}
			++count;
		}
		CyDelayUs(5);
	}
#endif
    
	// Set high the chip select.
#if 0
    //while ( 0 == (`$SPI_INSTANCE`_ReadTxStatus() & (`$SPI_INSTANCE`_STS_SPI_IDLE | `$SPI_INSTANCE`_STS_SPI_DONE)) );
    while ( 0 == (`$SPI_INSTANCE`_ReadTxStatus() & `$SPI_INSTANCE`_STS_SPI_IDLE ) );
#else
	do {
		status = `$SPI_INSTANCE`_ReadTxStatus() & (`$SPI_INSTANCE`_STS_SPI_IDLE|`$SPI_INSTANCE`_STS_SPI_DONE);
	} while ( status == 0);
#endif
    
	`$CS_INSTANCE`_Write(0xFF);
	`$SPI_INSTANCE`_ClearFIFO();
#else // `$INSTANCE_NAME`_Send (SCB Mode)
	int count = 0;
//	uint8 status;
	
	// wait for SPI operations to complete
	while ( `$SPI_INSTANCE`_SpiIsBusBusy() != 0) {
		CyDelay(1);
	}
		
	// set write bit in the control phase data
	block_select = block_select | ((write != 0) ? 0x04 : 0); 
	// select the data mode based on the block length
	if ( 1 ==  len ) {
		block_select |= 0x01;
	} else if ( 2 == len ) {
		block_select |= 0x02;
	} else if ( 4 == len ) {
		block_select |= 0x03;
	}
    
    #if 0
        block_select |= len;    
    #endif
	
	`$CS_INSTANCE`_Write(~(`$INSTANCE_NAME`_CS_MASK));
	// send the address phase data
	`$SPI_INSTANCE`_SpiUartWriteTxData( HI8(offset) );
	`$SPI_INSTANCE`_SpiUartWriteTxData( LO8(offset) );
	// send the control phase data
	`$SPI_INSTANCE`_SpiUartWriteTxData( block_select );

	// clear data read during the previous SPI write.  FIrst, wait for data
	// to arrive in the RX fifo (if not has yet been received), then, read the
	// data and wait for 3 data elements to be read (the length of the header
	// sent during the address and control phase of the protocol
	count = 3;
	while ( (count != 0) || (`$SPI_INSTANCE`_SpiUartGetRxBufferSize() ) ) {
		if (`$SPI_INSTANCE`_SpiUartGetRxBufferSize() ) {
			`$SPI_INSTANCE`_SpiUartReadRxData();
			count = (count == 0)? 0 : count - 1;
		}
		CyDelayUs(5);
	}
 
	// Now that the Receive FIFO has been flushed, send data through
	// the SPI port and wait for the receive buffer to contain data. Once the
	// receive buffer contains data, read the data and store it in to the
	// buffer.
	count = 0;
	while ( count < len ) {
		
		if ( `$SPI_INSTANCE`_SpiUartGetTxBufferSize() < 4 ) {
            
			`$SPI_INSTANCE`_SpiUartWriteTxData( (write==1) ? buffer[count] : 0xFF );
			while (`$SPI_INSTANCE`_SpiUartGetRxBufferSize() == 0);
            
			if ( write == 0 ) {
				buffer[count] = `$SPI_INSTANCE`_SpiUartReadRxData();
			} else {
				`$SPI_INSTANCE`_SpiUartReadRxData();
			}
            
			++count;
		}
        
		CyDelayUs(5);
	}
	
	// Set high the chip select.
	while (`$SPI_INSTANCE`_SpiIsBusBusy() != 0) {
		CyDelay(1);
	}
    
	`$CS_INSTANCE`_Write(0xFF);
	`$SPI_INSTANCE`_SpiUartClearRxBuffer();
	`$SPI_INSTANCE`_SpiUartClearTxBuffer();
#endif
}

/**
 * @brief
 *
 * @param 
 *
 * @return
 */
uint16_t `$INSTANCE_NAME`_RxDataReady( uint8_t socket )
{
	uint16_t first = 0;
    uint16_t second = 0;
	
	// quit on invalid sockets
	if ( `$INSTANCE_NAME`_SOCKET_BAD(socket) ) {
        return 0;
    }

	do {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_RX_RSR, `$INSTANCE_NAME`_SOCKET_BASE(socket), 0, (uint8*) &first, 2);
		if ( 0 != first ) {
			`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_RX_RSR, `$INSTANCE_NAME`_SOCKET_BASE(socket), 0,(uint8*) &second, 2);
		}
	} while (first != second );
	
	return CYSWAP_ENDIAN16(second);
}

/**
 * @brief
 *
 * @param 
 *
 * @return
 */
uint16_t `$INSTANCE_NAME`_TxBufferFree( uint8_t socket )
{
	uint16_t first = 0;
    uint16_t second = 0;
	
	// quit on invalid sockets
	if (`$INSTANCE_NAME`_SOCKET_BAD(socket)) {
        return 0;
    }

	do {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_TX_FSR, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,(uint8*)&first,2);
		if ( 0 != first ) {
			`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_TX_FSR, `$INSTANCE_NAME`_SOCKET_BASE(socket),0,(uint8*)&second,2);
		}
	} while ( first != second );
	
	return CYSWAP_ENDIAN16(second);
}

/**
 * @brief
 *
 * @param 
 *
 * @return
 */
void `$INSTANCE_NAME`_Reset( void )
{
	// issue a mode register reset to the W5500 in order to set default
	// register contents for the chip.
	uint8_t status = 0x80;
    
#if 0
	`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_MODE,`$INSTANCE_NAME`_BLOCK_COMMON,1, &status, 1);
    // Wait for the mode register to clear the reset bit, thus indicating
	// that the reset command has been completed.
	do {
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_REG_MODE, `$INSTANCE_NAME`_BLOCK_COMMON, 0, &status, 1);
	}
	while ( (status & 0x80) != 0 );
#else
	`$INSTANCE_NAME`_WriteSingle(`$INSTANCE_NAME`_REG_MODE, `$INSTANCE_NAME`_BLOCK_COMMON, status);
    // Wait for the mode register to clear the reset bit, thus indicating that the reset command has been completed.
    do {
        `$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_REG_MODE, `$INSTANCE_NAME`_BLOCK_COMMON, 0, &status, 1);
    } while ( 0 != ( status & 0x80 ) );
#endif
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
cystatus `$INSTANCE_NAME`_Init( uint8_t* gateway, uint8_t* subnet, uint8_t* mac, uint8_t *ip )
{
    uint8_t socket = 0;
	//int socket = 0;
	uint8_t socket_cfg[14] = { `$INSTANCE_NAME`_SOCKET_MEM, `$INSTANCE_NAME`_SOCKET_MEM, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t config[18] = {0};

	// build chip initialization from the default strings set in the configuration dialog
	for ( socket = 0; socket < 8; ++socket ) {
		`$INSTANCE_NAME`_socketStatus[socket] = `$INSTANCE_NAME`_SOCKET_AVAILABLE;
		// Send socket register setup to the chip
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_SREG_RXBUF_SIZE, `$INSTANCE_NAME`_SOCKET_BASE(socket), 1, &socket_cfg[0], 14);
	}
	
	if ( NULL != gateway ) {
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_REG_GAR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, gateway, 4);
		`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_REG_GAR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 4 );
		for ( socket = 0; (socket < 4) && ( gateway[socket] == config[socket] ); ++socket );
		if (socket < 4) {
            return CYRET_BAD_DATA;
        }
	}
	
	if ( NULL != subnet ) {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SUBR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, subnet, 4);
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SUBR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 4);
		for ( socket = 0; (socket < 4) && ( subnet[socket] == config[socket] ); ++socket );
		if ( socket < 4 ) {
            return CYRET_BAD_DATA;
        }
	}		

	if ( NULL != mac ) {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SHAR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, mac, 6);
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SHAR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 6);
		for ( socket = 0; (socket < 6) && ( mac[socket] == config[socket] ); ++socket);
		if ( socket < 6 ) {
            return CYRET_BAD_DATA;
        }
	}		

	if ( NULL != ip ) {
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SIPR, `$INSTANCE_NAME`_BLOCK_COMMON, 1, ip, 4);
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SIPR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, config, 4);
		for ( socket = 0; (socket < 4) && ( ip[socket] == config[socket] ); ++socket );
		if ( socket < 4 ) {
            return CYRET_BAD_DATA;
        }
	}		
	
	return CYRET_SUCCESS;
}

/**
 * \brief Initialize the W5500 with the default settings configured in Creator
 * 
 * This function calls the StartEx function to configure the W550 device using
 * the default setting as setup by the user in the configuration dialog in the
 * Creator tools.
 */
cystatus `$INSTANCE_NAME`_Start( void )
{
	return `$INSTANCE_NAME`_StartEx("`$GATEWAY`","`$SUBNET`","`$MAC`","`$IP`");
}

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
	uint32_t timeout = 0;
	uint32_t gar = 0;
    uint32_t subr = 0;
    uint32_t sipr = 0;
	uint8_t mac_array[6] = {0};
    uint8_t result = 0;
	uint8_t *m_gateway = NULL;
    uint8_t *m_subnet = NULL;
    uint8_t *m_mac = NULL;
    uint8_t *m_ip = NULL;
    
    // Start the SPI component
    `$SPI_INSTANCE`_Start();
    // Set CSN pin high
    `$CS_INSTANCE`_Write(0xFF);
    
	// Wait for initial power-on PLL Lock, and issue a device reset
	// to initialize the registers
	CyDelay(`$INSTANCE_NAME`_RESET_DELAY);
	
	// use IoT utlity functions to convert from ASCII data to binary data
	// used by the driver, and store it in the ChipInfo for later use.
	if ( (strlen( gateway ) > 0) && ( NULL != gateway ) ) {
		gar = `$INSTANCE_NAME`_ParseIP( gateway );
		m_gateway = (uint8*) &gar;
	} else {
		m_gateway = NULL;
	}
	
	if( ( strlen( subnet ) > 0) && ( NULL != subnet ) ) {
		subr = `$INSTANCE_NAME`_ParseIP( subnet );
		m_subnet = (uint8*) &subr;
	}  else {
		m_subnet = NULL;
	}
	
	if ( (strlen(mac) > 0) && ( NULL != mac ) ) {
		`$INSTANCE_NAME`_ParseMAC( mac, mac_array );
		m_mac = mac_array;
	} else {
		m_mac = NULL;
	}
	
	if ( (strlen(ip) > 0) && ( NULL != ip ) ) {
		sipr = `$INSTANCE_NAME`_ParseIP(ip);
		m_ip = (uint8*) &sipr;
	} else {
		m_ip = NULL;
	}
	
	// Attempt to initialize the device for approx. 1 second. If after
	// 1 second there is still a verification failure, return a timeout
	// error, otherwise return success!
	do {	
		`$INSTANCE_NAME`_Reset();
        
		result = `$INSTANCE_NAME`_Init( m_gateway, m_subnet, m_mac, m_ip );
        
		if ( CYRET_SUCCESS != result ) {
			CyDelay(1);
			++timeout;
			result = CYRET_TIMEOUT;
		}
	} while ( ( result != CYRET_SUCCESS ) && ( timeout < 1000 ) );
	
	return result;
}

/**
 * @brief
 *
 * @param 
 * @return
 */
void `$INSTANCE_NAME`_GetMac( uint8_t* mac )
{
	`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SHAR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, mac, 6);
}

/**
 * @brief
 *
 * @param 
 * @return
 */
uint32_t `$INSTANCE_NAME`_GetIp( void )
{
	uint32 ipr = 0;
	`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_REG_SIPR, `$INSTANCE_NAME`_BLOCK_COMMON, 0, (uint8*) &ipr, 4);
	return ipr;
}

/**
 * @brief
 *
 * @param 
 * @param
 * @param
 *
 * @return
 */
uint16_t `$INSTANCE_NAME`_GetTxLength( uint8_t socket, uint16_t len, uint8_t flags )
{
	uint16_t tx_length = 0;
    uint16_t max_packet = 0;
	uint8_t buf_size = 0;
	
	if ( `$INSTANCE_NAME`_SOCKET_BAD( socket ) ) {
        return 0;
    }
	
	tx_length = `$INSTANCE_NAME`_TxBufferFree( socket );
    
	if ( ( tx_length < len ) && ( 0 != ( flags & `$INSTANCE_NAME`_TXRX_FLG_WAIT ) ) ) {
		
        // there is not enough room in the buffer, but the caller requested
		// this to block until there was free space. So, check the memory
		// size to determine if the tx buffer is big enough to handle the
		// data block without fragmentation.
		`$INSTANCE_NAME`_Send(`$INSTANCE_NAME`_SREG_TXBUF_SIZE, `$INSTANCE_NAME`_SOCKET_BASE( socket ), 0, &buf_size, 1);
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
	
	return tx_length;
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
cystatus `$INSTANCE_NAME`_WriteTxData( uint8_t socket, uint8_t *buffer,
                                        uint16_t tx_length, uint8_t flags )
{
	uint16_t ptr = 0;
	cystatus result;
	
	// The length of the Tx data block has now been determined, and can be
    // copied in to the W5500 buffer memory.
	// First read the pointer, then write data from the pointer forward,
    // lastly update the pointer and issue the SEND command.
	`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_SREG_TX_WR, `$INSTANCE_NAME`_SOCKET_BASE( socket ), 0, (uint8*) &ptr, 2 );
    ptr = CYSWAP_ENDIAN16( ptr );
	`$INSTANCE_NAME`_Send( ptr, `$INSTANCE_NAME`_TX_BASE( socket ), 1, buffer , tx_length );
	ptr += tx_length;
	ptr = CYSWAP_ENDIAN16( ptr );
	`$INSTANCE_NAME`_Send( `$INSTANCE_NAME`_SREG_TX_WR, `$INSTANCE_NAME`_SOCKET_BASE( socket ), 1, (uint8*) &ptr, 2 );
	
	`$INSTANCE_NAME`_ExecuteSocketCommand( socket, `$INSTANCE_NAME`_CR_SEND );
	result = `$INSTANCE_NAME`_SocketSendComplete( socket );
	
	if ( (flags & `$INSTANCE_NAME`_TXRX_FLG_WAIT) != 0 ) {
		// block until send is complete
		do {
			CyDelay(1);
			result = `$INSTANCE_NAME`_SocketSendComplete( socket );
		} while( (result != CYRET_FINISHED) && (result != CYRET_CANCELED) );
	}
	
	return result;
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
void `$INSTANCE_NAME`_Write( uint16_t offset, uint8_t block_select,
                                uint8_t* data, size_t size )
{
#if 1
	// set write bit in the control phase data
	//block_select |= 0x04; 
    
	// select the data mode based on the block length    
    switch ( size ) {
    case 1:
        block_select |= 0x05; // 0x04 | 0x01
        break;
    case 2:
        block_select |= 0x06; // 0x04 | 0x02
        break;
    case 4:
        block_select |= 0x07; // 0x04 | 0x03
        break;
    default:
        break;
    }
	
    // Set /SS low
	`$CS_INSTANCE`_Write(~(`$INSTANCE_NAME`_CS_MASK));
    
	// send the address phase data
	`$SPI_INSTANCE`_WriteTxData( HI8(offset) );
	`$SPI_INSTANCE`_WriteTxData( LO8(offset) );
    
	// send the control phase data
	`$SPI_INSTANCE`_WriteTxData( block_select );
    
	// clear data read during the previous SPI write.  FIrst, wait for data
	// to arrive in the RX fifo (if not has yet been received), then, read the
	// data and wait for 3 data elements to be read (the length of the header
	// sent during the address and control phase of the protocol
    //`$SPI_INSTANCE`_ClearRxBuffer();
    //`$SPI_INSTANCE`_ClearTxBuffer();
	 
	// Now that the Receive FIFO has been flushed, send data through
	// the SPI port and wait for the receive buffer to contain data. Once the
	// receive buffer contains data, read the data and store it in to the
	// buffer.
    for ( uint8_t count = 0; count < size; count++) {
	    `$SPI_INSTANCE`_WriteTxData( data[count] );
    }
    
    while ( 0 == ( `$SPI_INSTANCE`_ReadTxStatus() & `$SPI_INSTANCE`_STS_SPI_IDLE ) );
    
	// Set high the chip select.
	`$CS_INSTANCE`_Write(0xFF);
#else
    // https://olduino.wordpress.com/2014/11/28/he-says-she-says-incremental-success-with-the-w5500/
    // Variable Length Write to Common or Socket areas
    // opcode is xxxx x100
    // xxxxx is:
    // 00000 for Common area
    // 00001 Socket 0 register
    // 00010 Socket 0 Tx buffer
    
    `$SPI_INSTANCE`_ClearRxBuffer();
    `$SPI_INSTANCE`_ClearTxBuffer();
	
	// set write bit in the control phase data
	block_select |= 0x04;
    block_select |= 0x01;
	
    // /SS active
    `$CS_INSTANCE`_Write( `$INSTANCE_NAME`_CS_ASSERT );
	//`$CS_INSTANCE`_Write(~(`$INSTANCE_NAME`_CS_MASK));
    
	// send the address phase data
	`$SPI_INSTANCE`_WriteTxData( HI8(offset) );
	`$SPI_INSTANCE`_WriteTxData( LO8(offset) );
    
	// send the control phase data - Send wiznet write opcode
	`$SPI_INSTANCE`_WriteTxData( block_select );
    // Write the actual data
    `$SPI_INSTANCE`_WriteTxData( data );

    // Wait for the transaction to be finished
    while ( 0 == (`$SPI_INSTANCE`_ReadTxStatus() & `$SPI_INSTANCE`_STS_SPI_IDLE ) );
    
	// Set high the chip select.
    //`$CS_INSTANCE`_Write( `$INSTANCE_NAME`_CS_DEASSERT );
    `$CS_INSTANCE`_Write(0xFF);
#endif
}

/**
 * @brief
 *
 * @param 
 * @param
 * @param
 *
 * @return
 */
void `$INSTANCE_NAME`_WriteSingle( uint16_t offset, uint8_t block_select, uint8_t data )
{
    `$SPI_INSTANCE`_ClearRxBuffer();
    `$SPI_INSTANCE`_ClearTxBuffer();
    
	// set write bit in the control phase data
	//block_select |= 0x04; 
    
	// select the data mode based on the block length
    block_select |= 0x05; // 0x04 | 0x01
	
    // Set /SS low
	`$CS_INSTANCE`_Write( 0 );
    
	// send the address phase data
	`$SPI_INSTANCE`_WriteTxData( HI8(offset) );
	`$SPI_INSTANCE`_WriteTxData( LO8(offset) );
    
	// send the control phase data
	`$SPI_INSTANCE`_WriteTxData( block_select );
    
	// clear data read during the previous SPI write.  FIrst, wait for data
	// to arrive in the RX fifo (if not has yet been received), then, read the
	// data and wait for 3 data elements to be read (the length of the header
	// sent during the address and control phase of the protocol
	 
	// Now that the Receive FIFO has been flushed, send data through
	// the SPI port and wait for the receive buffer to contain data. Once the
	// receive buffer contains data, read the data and store it in to the
	// buffer.
	`$SPI_INSTANCE`_WriteTxData( data );
    
    //while ( 0 == ( `$SPI_INSTANCE`_ReadTxStatus() & `$SPI_INSTANCE`_STS_SPI_DONE ) );
    
    CyDelayUs(5);
	// Set high the chip select.
	`$CS_INSTANCE`_Write( 1 );
    
    // https://olduino.wordpress.com/2014/11/28/he-says-she-says-incremental-success-with-the-w5500/
    // Variable Length Write to Common or Socket areas
    // opcode is xxxx x100
    // xxxxx is:
    // 00000 for Common area
    // 00001 Socket 0 register
    // 00010 Socket 0 Tx buffer
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
void `$INSTANCE_NAME`_Read( uint16_t offset, uint8_t block_select, uint8_t* data, size_t size )
{
    // Variable Length Read to Common or Socket areas
    // opcode is xxxx x000
    // xxxxx is:
    // 00000 for Common area
    // 00001 Socket 0 register
    // 00010 Socket 0 Tx buffer    
}

/** @} */
/* [] END OF FILE */

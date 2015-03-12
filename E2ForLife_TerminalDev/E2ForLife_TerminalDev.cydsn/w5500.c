/* ------------------------------------------------------------------------ */

#include <cylib.h>
#include <cytypes.h>

#include <ETH_CSN.h>
#include <SPI0.h>

#include "w5500.h"
#include "iot.h"

/* ------------------------------------------------------------------------ */
/* Macros & Function Defines */
/*
 * W5500 REgister Map
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
 * timeout used to prevent command acceptance deadlock
 */
#define W5500_CMD_TIMEOUT         ( 125 )

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

/**
 * constant conversion lookup table to convert socket number
 * to register block bank in the W5500 device.
 */
const uint8 w5500_socket_reg[8] = {
	W5500_BLOCK_S0_REG,
	W5500_BLOCK_S1_REG,
	W5500_BLOCK_S2_REG,
	W5500_BLOCK_S3_REG,
	W5500_BLOCK_S4_REG,
	W5500_BLOCK_S5_REG,
	W5500_BLOCK_S6_REG,
	W5500_BLOCK_S7_REG
};
/**
 * constant conversion lookup table to convert socket number
 * to TX Memory bank in the W5500 device.
 */
const uint8 w5500_socket_tx[8] = {
	W5500_BLOCK_S0_TXB,
	W5500_BLOCK_S1_TXB,
	W5500_BLOCK_S2_TXB,
	W5500_BLOCK_S3_TXB,
	W5500_BLOCK_S4_TXB,
	W5500_BLOCK_S5_TXB,
	W5500_BLOCK_S6_TXB,
	W5500_BLOCK_S7_TXB
};
/**
 * constant conversion lookup table to convert socket number
 * to register block bank in the W5500 device.
 */
const uint8 w5500_socket_rx[8] = {
	W5500_BLOCK_S0_RXB,
	W5500_BLOCK_S1_RXB,
	W5500_BLOCK_S2_RXB,
	W5500_BLOCK_S3_RXB,
	W5500_BLOCK_S4_RXB,
	W5500_BLOCK_S5_RXB,
	W5500_BLOCK_S6_RXB,
	W5500_BLOCK_S7_RXB
};
/* ------------------------------------------------------------------------ */
#define W5500_SOCKET_OPEN           ( 1 )
#define W5500_SOCKET_AVAILALE       ( 0 )

uint8 w5500_SOCKET_TABLE[8];

/* ------------------------------------------------------------------------ */
/**
 * \brief W5500 interface protocol write
 * \param offset address offset for the buffer access
 * \param block_select register block of the W5500 to access
 * \param buffer pointer to the data to write
 * \param len length of the data to send
 */
void w5500_Send(uint16 offset, uint8 block_select, uint8 write, uint8 *buffer, uint16 len)
{
	uint8 status;
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
	ETH_CSN_Write(0);
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
			status = SPI0_ReadRxData();
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
				status = SPI0_ReadRxData();
			}
			++count;
		}
		CyDelayUs(5);
	}
	/* Turn off the chip select. */
	do {
		status = SPI0_ReadTxStatus() & (SPI0_STS_SPI_IDLE|SPI0_STS_SPI_DONE);
	} while ( status == 0);
	ETH_CSN_Write(1);
	SPI0_ClearFIFO();
}
/* ------------------------------------------------------------------------ */
uint8 w5500_bSendControlReg( uint16 adr, uint8 value, uint8 write)
{
	w5500_Send(adr,W5500_BLOCK_COMMON,write,&value,1);
	return value;
}
uint16 w5500_wSendControlReg( uint16 adr, uint16 value, uint8 write)
{
	value = CYSWAP_ENDIAN16( value );
	w5500_Send(adr,W5500_BLOCK_COMMON,write,(uint8)&value,2);
	value = CYSWAP_ENDIAN16( value );
	return value;
}
uint32 w5500_lSendControlReg(uint16 adr, uint32 value, uint8 write)
{
	value = CYSWAP_ENDIAN32( value );
	w5500_Send(adr,W5500_BLOCK_COMMON,write, (uint8*)&value,4);
	value = CYSWAP_ENDIAN32( value );
	return value;
}
/* ------------------------------------------------------------------------ */

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
	
	w5500_Send(W5500_SREG_CR, w5500_socket_reg[socket], 1, &cmd, 1);
	timeout = 0;
	do {
		CyDelay(1);
		w5500_Send( W5500_SREG_CR, w5500_socket_reg[socket], 0, &result, 1);
		++timeout;
	}
	while ( (result != 0) && (timeout < W5500_CMD_TIMEOUT) );
	
	return result;
}
/* ======================================================================== */
/* End Section */
#endif
void w5500_Start( void )
{
	int socket;
	uint8 chip_config[18];
	uint32 ip;
	uint8* pip;
	
	pip = (uint8*)&ip;
	/*
	 * issue a mode register reset to the W5500 in order to set default
	 * register contents for the chip.
	 */
	w5500_bSendControlReg(W5500_REG_MODE, 0x80, 1);
	/*
	 * build chip initialization from the defaultl strings set in the
	 * configuration dialog
	 */
	for (socket = 0; socket < 8; ++socket) {
		w5500_SOCKET_TABLE[ socket ] = W5500_SOCKET_AVAILALE;
	}
	/* Build gateway address ppacket */
	ip = IOT_ParseIP("192.168.1.1");
	chip_config[0] = pip[0];
	chip_config[1] = pip[1];
	chip_config[2] = pip[2];
	chip_config[3] = pip[3];
	/* DEfault subnet mask */
	chip_config[4] = 0xFF;
	chip_config[5] = 0xFF;
	chip_config[6] = 0xFF;
	chip_config[7] = 0;
	/* Default hardware address */
	IOT_ParseMAC("00-DE-AD-BE-EF-00", &chip_config[8] );
	/* Default fixed IP address for the chip */
	ip = IOT_ParseIP("192.168.1.101");
	chip_config[14] = pip[0];
	chip_config[15] = pip[1];
	chip_config[16] = pip[2];
	chip_config[17] = pip[3];
	CyDelay(50);
	w5500_Send(W5500_REG_GAR,W5500_BLOCK_COMMON,1,&chip_config[0], 18);	
}
/* ------------------------------------------------------------------------ */
/* [] END OF FILE */

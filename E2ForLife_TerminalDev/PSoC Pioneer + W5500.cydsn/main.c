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
#include <project.h>

//#include "w5500.h"

void ETH_PutChar( char ch );
uint16 ETH_GetChar( void );
void ETH_PutString( const char *str );

char str[256];
uint8 udpHeader[19];
uint8 buffer[255];

/* ------------------------------------------------------------------------ */
int main()
{
	uint8 socket;
	uint16 data;
	uint8 mac[6];
	uint32 ip;
	
	// Wait for IMP reset monitor on the WizIOShield-A board to come out of reset
	CyDelay(560);
	
	/*
	 * Initialize the hardware perepherals used to communicate
	 */
	ETH_CSN_Write(1);
	SPI0_Start();
	RG_Start();
	B_Start();
	B_WritePulse0(255);
	RG_WritePulse0(255);
	RG_WritePulse1(255);
	
	/* Initialize the w5500 */
	if (w5500_Start() != CYRET_SUCCESS) {
		for(;;);
		RG_WritePulse0(0);
		RG_WritePulse1(255);
		B_WritePulse0(255);
	}
	
	socket = w5500_UdpOpen(8800);

    /* CyGlobalIntEnable; */ /* Uncomment this line to enable global interrupts. */
    for(;;)
    {
		data = w5500_UdpReceive(socket,udpHeader,buffer,255,0);
		if (data > 0) {
			RG_WritePulse0(~buffer[0]);
			RG_WritePulse1(~buffer[1]);
			B_WritePulse0(~buffer[2]);
		}
    }
}

/* [] END OF FILE */

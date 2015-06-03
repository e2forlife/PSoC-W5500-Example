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

char str[256];
uint8 udpHeader[19];
uint8 buffer[255];

/* ------------------------------------------------------------------------ */
int main()
{
	uint8 socket;
	uint16 data;
	
	// Wait for IMP reset monitor on the WizIOShield-A board to come out of reset
	CyDelay(560);
	
	/*
	 * Initialize the hardware perepherals used to communicate
	 */
	ETH_CSN_Write(1);
	SPI0_Start();
	RED_Write(1);
	GREEN_Write(1);
	BLUE_Write(1);
	
	/* Initialize the w5500 */
	if (w5500_Start() != CYRET_SUCCESS) {
		RED_Write(0);
		for(;;);
	}
	
	socket = w5500_UdpOpen(8800);

    /* CyGlobalIntEnable; */ /* Uncomment this line to enable global interrupts. */
    for(;;)
    {
		data = w5500_UdpReceive(socket,udpHeader,buffer,255,0);
		if (data > 0) {
			RED_Write( (buffer[0]!=0)?0:1 );
			GREEN_Write( (buffer[1]!=0)?0:1 );
			BLUE_Write( (buffer[0]!=0)?0:1 );
		}
    }
}

/* [] END OF FILE */

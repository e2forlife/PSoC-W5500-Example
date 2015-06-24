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

int main()
{
	uint8 socket;
	
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	ETH_Start();
	
    /* CyGlobalIntEnable; */ /* Uncomment this line to enable global interrupts. */
    for(;;)
    {
        socket = ETH_TcpOpenServer(23);
		ETH_TcpWaitForConnection( socket );
		ETH_TcpPrint(socket, "Hello world!");
		ETH_SocketDisconnect( socket );
    }
}

/* [] END OF FILE */

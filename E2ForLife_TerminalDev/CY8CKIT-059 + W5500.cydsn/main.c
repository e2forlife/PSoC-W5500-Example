#include "project.h"

int main()
{
	uint8_t socket;
	
    CyGlobalIntEnable;
    
	ETH_Start();
	
    for(;;)
    {
        socket = ETH_TcpOpenServer(23);
		ETH_TcpWaitForConnection( socket );
		ETH_TcpPrint(socket, "Hello world!");
		ETH_SocketDisconnect( socket );
    }
}

/* [] END OF FILE */

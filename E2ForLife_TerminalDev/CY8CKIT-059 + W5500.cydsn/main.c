#include "project.h"

// Para habilitar el soporte a C11 en compiladores "antiguos"
// agregar -std=c11 a los comandos del compilador
// enums anonimos, creo son C11
enum {
    UNO,
    DOS
};

enum {
    ONE,
    TWO
};

int main()
{	
    CyGlobalIntEnable;
    
    UART_Start();
	ETH_Start();
    
    uint8_t socket;
    cystatus tcp_connection;
    
    while (1) {
        socket = ETH_TcpOpenServer( 8080 );
        if ( 0xFF == socket) {
            UART_PutString("Socket not openned.\r\n");
        } else {
            UART_PutString("Socket openned.\r\n");
        }
        
        tcp_connection = ETH_TcpWaitForConnection( socket );
        if ( CYRET_SUCCESS == tcp_connection ) {
            UART_PutString("Connection stablished.\r\n");
        } else { // CYRET_BAD_PARAM
            UART_PutString("We got an error.\r\n");
        }
        
    	ETH_TcpPrint(socket, "Hello world!");
        
        tcp_connection = ETH_SocketDisconnect( socket );
        if ( CYRET_BAD_PARAM == tcp_connection ) {
            UART_PutString("We got a problem.\r\n");
        } else { // CYRET_SUCCESS
            UART_PutString("Socket closed.\r\n");
        }
    }
}

/* [] END OF FILE */

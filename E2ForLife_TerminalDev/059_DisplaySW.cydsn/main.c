#include "project.h"

void static_html( uint8_t socket)
{
    ETH_TcpPrint(socket, "<html>");
    ETH_TcpPrint(socket, "</html>");
}

int main()
{	
    uint8_t socket;
    cystatus tcp_connection;
    cystatus eth_status;
    
    CyGlobalIntEnable;
    
    UART_Start();
    UART_PutString("Test W5500.\r\n");

    UART_PutString("Inicializando el Ethernet.\r\n");
    eth_status = ETH_Start();
    
    if ( CYRET_SUCCESS == eth_status ) {
        UART_PutString("Ethernet success.\r\n");
    } else { // CYRET_TIMEOUT
        UART_PutString("Ethernet timeout.\r\n");
    }

    UART_PutString("Abriendo puerto 8080.\r\n");
    socket = ETH_TcpOpenServer( 8080 );
    if ( 0xFF == socket) {
        UART_PutString("Socket not openned.\r\n");
    } else {
        UART_PutString("Socket openned.\r\n");
    }
    
    while (1) {
        
        UART_PutString("Waiting for connection.\r\n");
        tcp_connection = ETH_TcpWaitForConnection( socket );
        if ( CYRET_SUCCESS == tcp_connection ) {
            UART_PutString("Connection stablished.\r\n");
        } else { // CYRET_BAD_PARAM
            UART_PutString("We got an error.\r\n");
        }

        UART_PutString("Socket Status: ");
        tcp_connection = ETH_TcpConnected( socket );
        switch( tcp_connection ){
        case 0: // Socket not established
            UART_PutString("Socket not established.\r\n");
            break;
        case 0xFF:
            UART_PutString("Socket not openned.\r\n");
            break;
        case 0x80:
            UART_PutString("Socket Timeout.\r\n");
            break;
        case 0x01:
            UART_PutString("Socket Connection.\r\n");
            ETH_TcpPrint(socket, "PSoC Latinoamerica\r\nPSoC Rocks!");
            break;
        default:
            break;
        }
        
        ETH_TcpPrint(socket, "PSoC Latinoamerica\r\nPSoC Rocks!");
        
        tcp_connection = ETH_SocketDisconnect( socket );
        if ( CYRET_BAD_PARAM == tcp_connection ) {
            UART_PutString("We got a problem.\r\n");
        } else { // CYRET_SUCCESS
            UART_PutString("Socket closed.\r\n");
        }
    }
}

/* [] END OF FILE */

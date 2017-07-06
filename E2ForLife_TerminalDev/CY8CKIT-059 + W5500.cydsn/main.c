#include "project.h"

void W5500_Write(uint16_t addr, uint8_t offset, uint8_t data)
{
    ETH_CSN_Write(1);
    SPI_WriteTxData( HI8(addr) );
    SPI_WriteTxData( LO8(addr) );
    
    offset |= 0x05;
    SPI_WriteTxData( offset );
    
    SPI_WriteTxData( data );
    
    while( !( SPI_ReadTxStatus() & SPI_STS_SPI_DONE ) );
    
    ETH_CSN_Write(1);
}

int main()
{	
    uint8_t socket;
    cystatus tcp_connection;
    
    CyGlobalIntEnable;
    
    UART_Start();
    UART_PutString("Test W5500.\r\n");
    SPI_Start();

    if ( CYRET_SUCCESS == ETH_Start() ) {
        UART_PutString("CYRET_SUCCESS.\r\n");
    } else { // CYRET_TIMEOUT
        UART_PutString("CYRET_TIMEOUT.\r\n");
    }
    
    while (1) {

        socket = ETH_TcpOpenServer( 80 );
        if ( 0xFF == socket) {
            UART_PutString("Socket not openned.\r\n");
        } else {
            UART_PutString("Socket openned.\r\n");
        }
            UART_PutString("Waiting for connection.\r\n");
            tcp_connection = ETH_TcpWaitForConnection( socket );
            if ( CYRET_SUCCESS == tcp_connection ) {
                UART_PutString("Connection stablished.\r\n");
            } else { // CYRET_BAD_PARAM
                UART_PutString("We got an error.\r\n");
            }

            ETH_TcpPrint(socket, "PSoC Latinoamerica\r\nPSoC Rocks!");
            
            tcp_connection = ETH_SocketDisconnect( socket );
            if ( CYRET_BAD_PARAM == tcp_connection ) {
                UART_PutString("We got a problem.\r\n");
            } else { // CYRET_SUCCESS
                UART_PutString("Socket closed.\r\n");
            }
        
        CyDelay(2000);
    }
}

/* [] END OF FILE */

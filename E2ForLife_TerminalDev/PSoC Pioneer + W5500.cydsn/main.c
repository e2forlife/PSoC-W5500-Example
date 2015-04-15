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
/*
 * shell test driver
 */
#include "shell.h"
#include "w5500.h"

void ETH_PutChar( char ch );
uint16 ETH_GetChar( void );
void ETH_PutString( const char *str );

/*
 * define the shell commands and callback functions for the command executions
 * note that if a command string appears more than one time, the callback for
 * each instance will be called in the order that they appear
 */
shell_COMMAND commands[] =
{
	{"test",  NULL },
	{"test1", NULL },
	{"test2", NULL },
	/* ---------------------------------- */
	{"", NULL} /* END OF SHELL TERMINATOR */
};
/*
 * define the basic shell implementation.  API callbacks MUST be defined, however,
 * startup, shutdown, and prompt callbacks are optional.  the command list is
 * required, or the shell with crash
 */
shell psoc_sh =
{
	commands,
	NULL, /* Startup callback */
	NULL, /* Prompt callback */
	NULL, /* Shutdown callback */
	ETH_PutChar, /* PutChar API callback (required) */
	ETH_GetChar, /* GetChar API callback (required) */
	ETH_PutString  /* PutString API Callback (required) */
};
/* ------------------------------------------------------------------------ */
void ETH_PutChar( char ch )
{
	/* send a character through the W5500 */
}

uint16 ETH_GetChar( void )
{
	/* read one char from the W5500 */
}

void ETH_PutString( const char *str )
{
	/* send a bunch of data */
}
/* ------------------------------------------------------------------------ */
int main()
{
	uint8 socket;
		
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	/*
	 * Initialize the hardware perepherals used to communicate
	 */
	ETH_CSN_Write(1);
	SPI0_Start();
	BLUE_Write(0);
	RED_Write(1);
	GREEN_Write(1);
	/* Initialize the w5500 */
	w5500_Start();
	
	socket = w5500_TcpOpenServer( 23 );
	w5500_TcpWaitForConnection( socket );
	GREEN_Write(0);
	BLUE_Write(1);
	RED_Write(1);
	w5500_TcpPrint( socket, "Hello World!\r\n");
	
	while (w5500_SocketSendComplete(socket) == 0) {
		CyDelay(1);
	}
	
	w5500_SocketClose( socket, 1 );
	
	//shell_Start( &psoc_sh );
    /* CyGlobalIntEnable; */ /* Uncomment this line to enable global interrupts. */
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */

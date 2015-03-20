#include <project.h>
#include "shell.h"

#include <stdio.h>

/* ------------------------------------------------------------------------- */
/* Global data */
uint8 shell_buffer[256];
uint16 shell_rx_write;
char shell_cmd[25];

char shell_msg[256];

/*
 * global storage for API Calls to read/write data using the defined interface
 * that is desired for the shell
 */
void (*shell_putchar)( char );
char (*shell_getchar)( void );
void (*shell_putstring)( const char* );

/* ------------------------------------------------------------------------- */
/**
 * \brief read a line of data from the user that defines the command
 * \returns int the length of data stored in the shell io buffer.
 *
 * shell_GetLine() scans the input stream and stores valid character data
 * in to the shell IO buffer for later processing.  Backspaces are handled
 * by removing the previous character from the buffer, and sending a string
 * to the shell stdout to remove the character from the screen. the receipt
 * of a carrage return or newline terminates the input stream and returns
 * control to the shell main.
 *
 * The total number of bytes stored in the shell IO buffer are returned
 */
int shell_GetLine( void )
{
	uint8 done;
	uint16 ch;
	int len;
	char c;
	
	len = 0;
	shell_rx_write = 0;
	done = 0;
	do {
		ch = shell_getchar();
		if ( (ch&0xFF00) == 0 ) {
			c = tolower(ch&0x00FF);
			++len;
			if ( ( c == 127 ) || (c == '\b') ) {
				if (shell_rx_write > 0) {
					shell_rx_write = shell_rx_write - 1;
					shell_putstring("\b \b"); /* erase letter from screen */
				}
				shell_buffer[shell_rx_write] = 0;
			}
			else if ( (c == '\r') || (c == '\n') ) {
				shell_putstring("\r\n");
				if (shell_rx_write > 255) {
					shell_buffer[255] = 0;
					shell_rx_write = 255;
					done = 1;
				}
				else {
					shell_buffer[shell_rx_write] = 0;
					done = 1;
				}
			}
			else {
				shell_putchar(ch);
				shell_buffer[ shell_rx_write++ ] = ch;
				shell_buffer[shell_rx_write] = 0;
				done = 0;
			}
		}
	}
	while (!done);
	
	return len;
}
/* ------------------------------------------------------------------------- */
int shell_ParseLine(int offset, uint8 *cmd)
{
	int idx;
	int i;
	int found;
	/*
	 * remove leading whitespace from the data starting at the offset.
	*/
	idx = 0;
	i = offset;
	found = 0;
	while ( ( (shell_buffer[i] == ' ') || (shell_buffer[i]=='\t') ) && (shell_buffer[i] != 0) ) {
		++i;
	}
	/*
	 * copy data from the buffer to the command string starting at teh first
	 * non-blank entry until either a blank or an EOL is read.
	 */
	while ( (shell_buffer[i] != 0) && (!isblank( shell_buffer[i])) ) {
		cmd[idx++] = shell_buffer[i++];
		cmd[idx] = 0;
		found = 1;
	}
	return (found==0)?-1:i;
}
/* ------------------------------------------------------------------------- */
int shell_Start( shell *sh )
{
	int done;
	int offset;
	int idx;
	
	shell_void_func startup;
	shell_void_func shutdown;
	shell_void_func prompt;
	shell_COMMAND *cmd;
	shell_command_func func;
	/*
	 * pull out the callback functions for the startup/shutdown and command
	 * array for the shell operation. startup (when defined) is called before
	 * entering the shell, shutdown (when available) is called after the shell
	 * exits.
	 */
	startup = sh->startup;
	shutdown = sh->shutdown;
	prompt = sh->prompt;
	cmd = sh->cmd;
	/*
	 * assign the local callback function pointers for the interface used for
	 * the API calls to get and send data.
	 */
	shell_putchar = sh->putchar;
	shell_getchar = sh->getchar;
	shell_putstring = sh->putstring;
	/*
	 * If the API calls have not been initialized properly, return with
	 * an error code, and don't execute any code in teh shell
	 */
	if ( (shell_putchar == NULL) || (shell_getchar==NULL) || (shell_putstring == NULL) ) {
		return -1;
	}
	/*
	 * if there is shell startup code, execute it here.
	 */
	if (startup != NULL) startup();
	done = 0;
	do {
		idx = 0;
		offset = 0;
		if (prompt != NULL) prompt();
		shell_GetLine();
		offset = shell_ParseLine(offset, (uint8*)&shell_cmd[0]);
		func = NULL;
		while (( strlen(cmd[idx].command) > 0) && (done == 0) ) {
			if (strcmp(shell_cmd, cmd[idx].command) == 0) {
				/* found the command, so execute the function */
				func = cmd[idx].exec;
				if (func != NULL ) {
					done = func( offset );
				}
				else {
					shell_putstring("\r\nSorry, but command \x1b[97m");
					shell_putstring(shell_cmd);
					shell_putstring("\x1b[37m has not yet been implemented.\r\n");
					shell_putstring("\x1b[36mPlease contact your friendly software engineering department\r\n");
					shell_putstring("for more details.\x1b[37m");
				}
			}
			++idx;
		}
		if (func == NULL) {
			shell_putstring("\r\n\r\n\x1b[37mUnknown Command \x1b[97m");
			shell_putstring(shell_cmd);
			shell_putstring("\x1b[37m.\r\n");
			for(idx=0;idx<(int)strlen(shell_cmd);++idx) {
				sprintf(shell_msg,"\x1b[34m[\x1b[97m%02X\x1b[34m]",shell_cmd[idx]);
				shell_putstring(shell_msg);
			}
			shell_putstring("\x1b[37m\r\n\r\n");
		}
	}
	while (!done);
	
	if (shutdown != NULL) shutdown();
	return done;
}
/* ------------------------------------------------------------------------- */
/* [] END OF FILE */

#if !defined(shell_H)
	#define shell_H

#include <cytypes.h>

typedef uint8 (*shell_command_func)(int);
typedef void (*shell_void_func)( void );

typedef struct
{
	char command[16];
	shell_command_func exec;
} shell_COMMAND;

typedef struct {
	shell_COMMAND *cmd;
	shell_void_func      startup;
	shell_void_func      prompt;
	shell_void_func      shutdown;
	void (*putchar)( char );
	uint16 (*getchar)( void );
	void (*putstring)(const char*);
} shell;

int shell_Start( shell *sh );
int shell_GetLine( void );
int shell_ParseLine(int offset, uint8 *cmd);

#endif
/* [] END OF FILE */

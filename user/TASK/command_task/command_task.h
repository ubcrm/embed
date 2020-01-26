#ifndef CMD_TASK_H
#define CMD_TASK_H

#include "stm32f4xx.h"

#define COMMAND_LENGTH 5
#define RESPONSE_LENGTH 50
#define MAX_NUM_COMMANDS 20
#define MAX_COMMAND_LINE 15

typedef struct {
	uint16_t name[COMMAND_LENGTH]; //Hard rule that all commands are 5 characters long
	//callback
	void (*callback) (uint16_t[10]); //The rest of the command line is the argument
	int num_args; //How many arguments do you need?
	char wrong_args[RESPONSE_LENGTH]; //You have 50 characters to say they're wrong.
} command;

typedef struct {
	uint16_t name[COMMAND_LENGTH];
	command exec;
} cmd_entry;

extern cmd_entry commands[MAX_NUM_COMMANDS];
//void USART6_IRQHandler(void);

#endif

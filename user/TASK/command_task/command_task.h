#ifndef CMD_TASK_H
#define CMD_TASK_H

#include "stm32f4xx.h"

typedef struct {
	uint16_t name[5]; //Hard rule that all commands are 5 characters long
	//callback
	void (*callback) ();
	int num_args; //How many arguments do you need?
	char wrong_args[50]; //You have 50 characters to say they're wrong.
} command;

typedef struct {
	uint16_t name[5];
	command exec;
} cmd_entry;

extern cmd_entry commands[20];

#endif
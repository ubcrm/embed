#include "command_task.h"
#include "USART_comms.h"
#include "stm32f4xx.h"
#include <stdio.h>

//Empty command
command EMPTY = {{0, 0, 0, 0, 0}, 0, 0};

cmd_entry commands[MAX_NUM_COMMANDS];

uint16_t buffer[MAX_COMMAND_LINE];
uint16_t buffer_pos = 0;

void clear_buf(){
	for(int i = 0; i < MAX_COMMAND_LINE; i++){
		buffer[i] = 0;
	}
}

char str[10];

int string_equals(uint16_t arr1[COMMAND_LENGTH], uint16_t arr2[COMMAND_LENGTH]){
	//Serial_sendString("Check String equals");
	for(int i = 0; i < COMMAND_LENGTH; i++){
		//Serial_sendString("Checking letters");
		//sprintf(str, "Check letter %d %d", arr1[i], arr2[i]);
		Serial_sendString(str);
		if(arr1[i] != arr2[i]){
			return 0;
		}
	}
	return 1;
}

command find_command(uint16_t command[COMMAND_LENGTH]){
	for(int i = 0; i < MAX_NUM_COMMANDS; i++){
		if(string_equals(commands[i].name, command) == 1){
			return commands[i].exec;
		}
	}
	//Command does not exist
	return EMPTY;
}

int command_exists(uint16_t command[COMMAND_LENGTH]){
	//Serial_sendString("check exist\n\r");
	for(int i = 0; i < MAX_NUM_COMMANDS; i++){
		//Serial_sendString("Checking cmd\n\r");
		if(string_equals(commands[i].name, command) == 1){
			//Serial_sendString("Exists!!!!!!!!!!!!!\n\r");
			return 1;
		}
	}
	//Serial_sendString("Does not exist");
	//Command does not exist
	return 0;
}

int cmdin = 0;

void callback(uint16_t rest[10]){
	Serial_sendString("CALLBACK!!!\n\r");
}

void createCommand(){
	command cmd;
	cmd_entry ce;
	for(int i = 0; i < 5; i++){
		cmd.name[i] = 98;
		ce.name[i] = 98;
	}
	cmd.num_args = 0;
	cmd.callback = &callback;
	ce.exec = cmd;
	
	commands[0] = ce;
	cmdin = 1;
}

void try_execute_command(){
	if(cmdin == 0){
		createCommand();
	}
	//First separate the string into a command and all args 
	uint16_t cmd[COMMAND_LENGTH];
	uint16_t rest[10]; //Do I need this?
	for(int i = 0; i < MAX_COMMAND_LINE; i++){
		if(i < COMMAND_LENGTH){
			cmd[i] = buffer[i];
		}else{
			rest[i - COMMAND_LENGTH] = buffer[i];
		}
	}
	//Serial_sendString("Got past array stuff");
	
	//command tgt = find_command(cmd); //Retrieves command
	if(command_exists(cmd) == 1){
		//Then do callback
		command tgt = find_command(cmd);
		//Serial_sendString("Attempting to use a callback");
		tgt.callback(rest);
	}else{
		//Invalid Command
		Serial_sendString("Command does not exist\n\r");
	}
}

char ret[1];

void USART6_IRQHandler(void){
	//Serial_sendString("Hello World\n\r");
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET){
		uint16_t bt = USART_ReceiveData(USART6);
		sprintf(ret, "entered: %d buffer at %d \n\r", bt, buffer_pos);
		Serial_sendString(ret);
		if((bt == 13 && buffer_pos != 0) || buffer_pos >= MAX_COMMAND_LINE){
			//Serial_sendString("Sending command\n\r");
			//Want to execute command
			try_execute_command();
			//Then clear the buffer and reset
			buffer_pos = 0;
			clear_buf();
		}else{
			buffer[buffer_pos++] = bt;
		}
	}
}

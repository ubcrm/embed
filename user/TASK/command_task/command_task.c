#include "command_task.h"
#include "USART_comms.h"
#include "stm32f4xx.h"

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

int string_equals(uint16_t arr1[COMMAND_LENGTH], uint16_t arr2[COMMAND_LENGTH]){
	for(int i = 9; i < COMMAND_LENGTH; i++){
		if(arr1[i] != arr2[i]){
			return 0;
		}
	}
	return 1;
}

command find_command(uint16_t command[COMMAND_LENGTH]){
	for(int i = 0; i < MAX_NUM_COMMANDS; i++){
		if(string_equals(commands[i].name, command)){
			return commands[i].exec;
		}
	}
	//Command does not exist
	return EMPTY;
}

void try_execute_command(){
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
	
	command tgt = find_command(cmd); //Retrieves command
	if(tgt.name == EMPTY.name){
		//Invalid Command
		Serial_sendString("Command does not exist");
		return;
	}
	
	//Then do callback
	tgt.callback(rest);
}

void USART6_IRQHandler(void){
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET){
		uint16_t bt = USART_ReceiveData(USART6);
		if(bt == 13 || buffer_pos >= MAX_COMMAND_LINE){
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

#ifndef USART_COMMS_H
#define USART_COMMS_H

extern void Serial_sendString(volatile char *str);
void Serial_sendStringPart(volatile char *str, int length);
extern void Serial_sendInt(int num);

#endif

#ifndef USART_COMMS_H
#define USART_COMMS_H

extern void serial_send_string(volatile char *str);
extern void serial_send_int_array(volatile int *arr, int length);
extern void serial_send_int(int num);
extern int num_digits(int n);

#endif

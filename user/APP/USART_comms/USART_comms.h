#ifndef USART_COMMS_H
#define USART_COMMS_H
#include "main.h"

static volatile char READY = 1;
static volatile char NOT_READY = 0;

extern void serial_send_string(volatile char *str);
extern void serial_send_int_array(volatile int *arr, int length);
extern void serial_send_int(int num);
extern int num_digits(int n);

extern void vision_send_string(volatile char *str);

typedef struct
{
    uint8_t num_filled;
    uint8_t packets[3];
    uint16_t integer;
    uint16_t decimal;
    uint8_t checksum;
    
} Gimbal_buffer;

#endif

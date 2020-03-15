#ifndef USART_COMMS_H
#define USART_COMMS_H

static char READY;
static char NOT_READY;

extern void serial_send_string(volatile char *str);
extern void serial_send_int_array(volatile int *arr, int length);
extern void serial_send_int(int num);
extern int num_digits(int n);

typedef struct
{
    uint8_t num_filled;
    uint8_t packets[3];
    uint16_t integer;
    uint16_t decimal;
    uint8_t checksum;
    
} Gimbal_buffer;

// A struct that holds all of the gimbal data
static Gimbal_buffer gimbal_buff;

#endif

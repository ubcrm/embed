#ifndef FRIC_H
#define FRIC_H
#include "main.h"

#define Fric_UP 1900
#define Fric_DOWN 1000
#define Fric_OFF 900

// user defines
#define Fric_INIT 50

extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif

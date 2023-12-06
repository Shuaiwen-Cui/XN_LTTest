#ifndef _LED_H
#define _LED_H

#include <stdbool.h>

void LED_Init(void);
void LED_Off(void);
void LED_RGB(bool ron, bool gon, bool bon);
void LED_R(bool on);
void LED_Rtoggle(void);
void LED_G(bool on);
void LED_Gtoggle(void);
void LED_B(bool on);
void LED_Btoggle(void);

#endif /* _LED_H */

#ifdef USB
#ifndef _VCOM_H
#define _VCOM_H

#include <stdbool.h>

void VCOM_Init(void);
void VCOM_Connect(bool blocking);
void VCOM_Disconnect(void);
void VCOM_Write(char *buf, int size);
void VCOM_Read(char *buf, int size, int *read);
void VCOM_PutChar(unsigned char x);
unsigned char VCOM_GetChar(void);

#endif /* _VCOM_H */
#endif

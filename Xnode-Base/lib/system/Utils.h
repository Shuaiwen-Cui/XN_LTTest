#ifndef _UTILS_H 
#define _UTILS_H

#include <stdint.h>

int Util_Init(void);

int Util_Wakeup(uint16_t *addrs, uint8_t *addrlen);
int Util_ReadVoltage(uint16_t *addrs, uint8_t addrlen);
int Util_ResetNodes(uint16_t *addrs, uint8_t addrlen);
int Util_Wakeup_Retrieve(uint16_t *addrs, uint8_t *addrlen);

#endif /* _UTILS_H */

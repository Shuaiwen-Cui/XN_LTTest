#ifndef _RELIABLECOMM_H
#define _RELIABLECOMM_H

#include <stdint.h>
#include "ReliableCommImpl.h"

typedef void (*rc_scallback_t)(uint16_t *addrs, uint8_t addrlen, uint8_t *data, int success);
typedef void (*rc_rcallback_t)(uint16_t src, uint16_t *addrs, uint8_t addrlen, uint8_t *data, uint32_t len, int success);

int ReliableComm_init(void);
int ReliableComm_reset(void);
bool ReliableComm_isBusy(void);
int ReliableComm_register(uint8_t id, rc_scallback_t scallback, rc_rcallback_t rcallback);
int ReliableComm_unregister(uint8_t id);
int ReliableComm_send(uint8_t id, const uint16_t *addrs, uint8_t addrlen, uint8_t *data, uint32_t len);

#endif /* _RELIABLECOMM_H */

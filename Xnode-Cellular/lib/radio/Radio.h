#ifndef _RADIO_H_
#define _RADIO_H_

#include <stdint.h>
#include <rf233-config.h>

#define BCAST_ADDR             (0xFFFF)
#define MAX_PAYLOAD_SIZE       (116)

int radio_init(void);
int radio_reset(void);
int radio_send(uint16_t dest, void *payload, uint8_t len);
int radio_recv(uint16_t *src, uint8_t *payload, uint8_t *len);

int radio_on(void);
int radio_off(void);
int radio_sleep(void);

int radio_set_short_addr(uint16_t addr);
int radio_get_channel(void);
int radio_set_channel(uint8_t ch);
int radio_get_txp(void);
int radio_set_txp(uint8_t txp);

#endif  /* _RADIO_H_ */

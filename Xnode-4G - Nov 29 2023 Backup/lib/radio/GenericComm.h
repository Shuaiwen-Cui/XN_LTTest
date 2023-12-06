#ifndef _GENERICCOMM_H
#define _GENERICCOMM_H

#include "Radio.h"

typedef struct {
	uint8_t id;
	uint8_t flag;
} __attribute__((packed)) gc_header_t;

#define GC_HEADER_SIZE      (sizeof (gc_header_t))
#define GC_MAX_PAYLOAD_SIZE (MAX_PAYLOAD_SIZE - GC_HEADER_SIZE)

typedef void (*gc_callback_t)(uint16_t src, void *payload, uint8_t len);

int GenericComm_init(void);
int GenericComm_register(uint8_t id, gc_callback_t callback);
int GenericComm_unregister(uint8_t id);
int GenericComm_send(uint8_t id, uint16_t dest, const uint8_t *payload, uint8_t len, uint8_t ack);
int GenericComm_bcast(uint8_t id, const uint8_t *payload, uint8_t len);

#endif /* _GENERICCOMM_H */

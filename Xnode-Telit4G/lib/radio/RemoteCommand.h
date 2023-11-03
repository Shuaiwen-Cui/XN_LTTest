#ifndef _REMOTECOMMAND_H
#define _REMOTECOMMAND_H

#include <stdbool.h>
#include <stdint.h>

typedef int (*rc_cmdfunc_t)(void* arg, uint32_t len);
typedef void (*rc_cmdsentcb_t)(uint16_t *targets, uint8_t tcount);
typedef void (*rc_respsentcb_t)(int success);
typedef void (*rc_cmdexeccb_t)(int success, void *retval, uint32_t len);

int RemoteCommand_init(void);
int RemoteCommand_register(uint8_t id, bool retval, rc_cmdfunc_t cmd, rc_cmdsentcb_t sent, rc_respsentcb_t resp, rc_cmdexeccb_t exec);
int RemoteCommand_unregister(uint8_t id);

// caller
int RemoteCommand_execute(uint8_t id, uint16_t *tgts, uint8_t tcnt, const void *args, uint32_t arglen);
int RemoteCommand_stop(uint8_t id);

// callee
int RemoteCommand_done(uint8_t id, int res, void *retval, uint32_t len);

#endif /* _REMOTECOMMAND_H */

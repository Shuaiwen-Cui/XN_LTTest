#ifndef _SENDSMSG_H
#define _SENDSMSG_H

#include "ReliableCommImpl.h"

typedef void (*sm_callback_t)(SSMInfo *s, int success);

int SendSMsg_init(sm_callback_t scb, sm_callback_t rcb);
int SendSMsg_reset(void);
bool SendSMsg_isBusy(void);
int SendSMsg_send(SSMInfo *s);
int SendSMsg_bcast(SSMInfo *s);
void TimerStop(void);
#endif /* _SENDSMSG_H */

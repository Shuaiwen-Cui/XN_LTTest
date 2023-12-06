#ifndef _SENDLDATA_H
#define _SENDLDATA_H

#include "ReliableCommImpl.h"

typedef void (*ld_callback_t)(int success);
typedef void (*ld_acallback_t)(void);

int SendLData_init(ld_callback_t scb, ld_callback_t rcb, ld_acallback_t acb);
int SendLData_reset(void);
bool SendLData_isBusy(void);
int SendLData_SLDIAlloc(SLDInfo *s);
int SendLData_send(void);
int SendLData_bcast(void);
	
#endif /* _SENDLDATA_H */

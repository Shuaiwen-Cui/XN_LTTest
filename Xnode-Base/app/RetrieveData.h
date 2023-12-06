#ifndef _RETRIEVEDATA_H
#define _RETRIEVEDATA_H

#include <lpc_types.h>

typedef struct {
	bool latestselect;	// order to select data - start from the latest (true) or from the earliest (false)
	uint8_t datacount;
} __attribute__((packed)) drmessage;

int RetrieveData_menu(void);
int RetrieveData_init(void);
int RetrieveData_start(void);
int RetrieveData_setparams(void);

#endif /* _RETRIEVEDATA_H */

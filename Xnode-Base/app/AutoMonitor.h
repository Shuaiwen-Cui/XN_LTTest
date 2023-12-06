#ifdef GATEWAY
#ifndef _AUTOMONITOR_H
#define _AUTOMONITOR_H

#include <stdint.h>

extern uint8_t AM_HOURS;

int AutoMonitor_menu(void);
int AutoMonitor_running(void);
int AutoMonitor_readcfg(void);
int AutoMonitor_writecfg(void);

#endif /* _AUTOMONITOR_H */
#endif

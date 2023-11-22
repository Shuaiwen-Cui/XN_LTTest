#ifndef _REMOTESENSING_H
#define _REMOTESENSING_H

#include <stdint.h>
#include <GlobalConstants.h>
#include <Sensing.h>

#ifdef FRA
#define DATACOUNTHRESHOLD 0			// For now 0 is the best option - do not change
int WriteToNAND(uint8_t *data, uint32_t* sdidx);
int WriteToSD(uint8_t *data, uint32_t* sdidx);
int PackAndSave(SensorData *sd);
void packSensorDataRAW(float **sd, uint32_t numchan, uint32_t numsam);
#endif
extern uint16_t rsnodes[MAX_NODES];
extern uint8_t rsncnt, rschannels;
extern uint16_t rstime, rsrate;
extern uint32_t data_index;

void RemoteSensing_menu(void);
int RemoteSensing_init(void);
int RemoteSensing_start(void);
int RemoteSensing_writedata(SensorData *sd, uint16_t nodeid, uint32_t index);
int write_acc(float *output[], uint32_t size);		///////// check the index
int RemoteSensing_setparams(void);

/*------- debug filter -----------*/
void read_int(int16_t *inputz, uint16_t length, uint8_t index);
void write_int(int32_t *outputz, uint32_t size, uint8_t index);
void read_float(float *inputz, uint16_t length, uint8_t index);
void write_float(float *outputz, uint32_t size, uint8_t index);
int XnodeCfg_setparams(void);
int XnodeCfgSend_init(void);
int XnodeCfgSend(uint16_t *addrs, uint8_t addrlen);
int XnodeConfig_Init(void);
void gettime(uint8_t *yy, uint8_t *momo, uint8_t *dd, uint8_t *hh, uint8_t *mm, uint8_t *ss);

#endif /* _REMOTESENSING_H */

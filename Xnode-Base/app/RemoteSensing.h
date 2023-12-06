#ifndef _REMOTESENSING_H
#define _REMOTESENSING_H

#include <stdint.h>
#include <GlobalConstants.h>
#include <Sensing.h>

typedef struct {
	float scale;
	float offset;
} __attribute__((packed)) calibinfo;


extern uint16_t rsnodes[MAX_NODES];
extern uint8_t rsncnt, rschannels;
extern uint16_t rstime, rsrate;
extern uint32_t data_index;
extern bool rsretrievedata, rsprintdata;
extern calibinfo rscalibinfo[5];

void RemoteSensing_menu(bool manual);
int RemoteSensing_init(void);
int RemoteSensing_start(void);
int RemoteSensing_writedata(SensorData *sd, uint16_t nodeid, uint32_t index);
int write_acc(float *output[], uint32_t size);		///////// check the index
int RemoteSensing_setparams(void);

void gettime(uint8_t *yy, uint8_t *momo, uint8_t *dd, uint8_t *hh, uint8_t *mm, uint8_t *ss);
void settime(uint8_t yy, uint8_t momo, uint8_t dd, uint8_t hh, uint8_t mm, uint8_t ss);
void getcalibinfo(uint8_t channel, float *scale, float *offset);
void setcalibinfo(uint8_t channel, float scale, float offset);

/*------- debug filter -----------*/
void read_int(int16_t *inputz, uint16_t length, uint8_t index);
void write_int(int32_t *outputz, uint32_t size, uint8_t index);
void read_float(float *inputz, uint16_t length, uint8_t index);
void write_float(float *outputz, uint32_t size, uint8_t index);
int XnodeCfg_setparams(void);
int XnodeCfgSend_init(void);
int XnodeCfgSend(uint16_t *addrs, uint8_t addrlen);
int XnodeConfig_Init(void);

#endif /* _REMOTESENSING_H */

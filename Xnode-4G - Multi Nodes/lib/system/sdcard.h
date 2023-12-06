#ifndef _SDCARD_H
#define _SDCARD_H

#ifndef FRA	// FRA version has more parameters in the xnode.cfg
typedef struct {
	uint16_t nodeid;
	uint8_t channel;
	uint8_t power;
	uint8_t trigsen;
} flashconfig_t;
#endif

#ifdef FRA
typedef struct {
	uint16_t nodeid;
	uint8_t channel;
	uint8_t power;
	uint8_t task1time;
	uint8_t task2time;
	uint8_t task3time;
	uint16_t adxlrange;
	uint16_t adxlthresholdact;
	uint16_t adxltimeact;
	uint16_t adxlthresholdinact;
	uint16_t adxltimeinact;
	uint16_t adxlfifonum;
	uint16_t remotesensingtime;
	uint16_t remotesensingrate;
	uint8_t remotesensingchannels;
	uint8_t remotesensingncnt;
	uint16_t remotesensingnodes[MAX_NODES];
	uint16_t blocklimit;	//TUADD: Limit of # of dirty block before erasing
	uint16_t senselowerlimit;	//TUADD: lower limit of sensing (ms)
	uint32_t senseupperlimit;	//TUADD: upper limit of sensing (ms)
	uint64_t phonenumber;
	uint16_t ftpsite1;
	uint16_t ftpsite2;
	uint16_t ftpsite3;
	uint16_t ftpsite4;	
	char ftpusername[30];	
	char ftppassword[30];	
	char ftpapn[30];	
} flashconfig_t;

bool read_nand2sdattemp(void);
void write_nand2sdattemp(bool nand2sdattemp);

#endif

int SDCard_ReWriteCN(flashconfig_t fc); // no init
int read_tosend(void);
int NAND2SD(void);
#endif

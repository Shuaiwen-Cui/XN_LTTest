#include <xnode.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_sdif.h>
#include <lpc43xx_sdmmc.h>
#include <lpc_sdmmc.h>
#include <nandflash_k9f1g08u0a.h>
#include <stdio.h>
#include <string.h>
#include <ff.h>
#include <inttypes.h>
#include <sdcard.h>
#include <rtc.h>

#define _DEBUG_               1
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if _DEBUG_
static const char* print_error(FRESULT res)
{
	switch (res) {
		case FR_OK:								return "Succeeded";
		case FR_DISK_ERR:					return "A hard error occured in the low level disk I/O layer";
		case FR_INT_ERR:					return "Assertion failed";
		case FR_NOT_READY:				return "The physical drive cannot work";
		case FR_NO_FILE:					return "Could not find the file";
		case FR_NO_PATH:					return "Could not find the path";
		case FR_INVALID_NAME:			return "The path name format is invalid";
		case FR_DENIED:						return "Acces denied due to prohibited access or directory full";
		case FR_EXIST:						return "Acces denied due to prohibited access";
		case FR_INVALID_OBJECT:		return "The file/directory object is invalid";
		case FR_WRITE_PROTECTED:	return "The physical drive is write protected";
		case FR_INVALID_DRIVE:		return "The logical drive number is invalid";
		case FR_NOT_ENABLED:			return "The volume has no work area";
		case FR_NO_FILESYSTEM:		return "There is no valid FAT volume on the physical drive";
		case FR_MKFS_ABORTED:			return "The f_mkfs() aborted due to any parameter error";
		case FR_TIMEOUT:					return "Could not get a grant to access the volume within defined period";
		case FR_LOCKED:						return "The operation is rejected according to the file shareing policy";
		case FR_NOT_ENOUGH_CORE:	return "LFN working buffer could not be allocated";
		case FR_TOO_MANY_OPEN_FILES:	return "Number of open files > _FS_SHARE";
		default:
			return "unknown";
	}
}
#endif

uint32_t wk_index;
extern uint8_t TrigSenIdx;
#define LINELEN               80
FATFS Fatfs;                  // File system object for each logical drive
int SD_ISIN = 1;              // assume as SD is plugged in, if it's not in, it's switched to 0 and never used again (skipped in fchir)

FRESULT open_append (
	FIL* fp,            /* [OUT] File object to create */
	const char* path    /* [IN]  File name to be opened */
)
{
	FRESULT fr;

	/* Opens an existing file. If not exist, creates a new file. */
	fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
	if (fr == FR_OK) {
		/* Seek to end of the file to append data */
		fr = f_lseek(fp, f_size(fp));
		if (fr != FR_OK)
			f_close(fp);
	}
	return fr;
}
// NAND flash

//Understanding Flash: Blocks, Pages and Program / Erases
//Blocks((1024 Block usable) are the smallest flash unit that can be erased. Remember that, it is really important.
//Each block contains a number of pages(64Pages/Block), which are the smallest unit that can be programmed (i.e. written to).
//Each pages can save 2048 bytes data.

bool sdready = false;

#ifndef FRA	// FRA version has more parameters in the xnode.cfg
int SDCard_Init(void)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	char line[LINELEN];
	uint32_t nodeid, channel, power, idx;
	int res = FAIL;
	flashconfig_t fc, *fcptr;
	uint8_t *flashbuf;

	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
	NandFlash_Init();
	if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash read success\r\n");
		fcptr = (flashconfig_t *)flashbuf;
		LOCAL_ADDR = fcptr->nodeid;
		RADIO_CHANNEL = fcptr->channel;
		RADIO_POWER = fcptr->power;
		TrigSenIdx = fcptr->trigsen;
		res = SUCCESS;
	} else {
		PRINTF("NAND Flash read failed\r\n");
		res = FAIL;
	}

	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object
	f_mount(0, &Fatfs);
	// go to config dir
	ret = f_chdir("/Xnode");
	if (ret) {
		PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		sdfree(flashbuf);
		if (ret == FR_NOT_READY)
		{
			SD_ISIN = 0;
		}
	return res;
	}

	// read config file
	ret = f_open(&file, "Xnode.cfg", FA_READ);
	if (ret) {
		sdready = false;
		PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		sdfree(flashbuf);
		return res;
	}

	// node id
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "NODEID = %u", &nodeid) != 1 || nodeid >= 0xFFFF) {
		sdready = false;
		PRINTF("Error reading node id\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	LOCAL_ADDR = (uint16_t)nodeid;
	PRINTF("NODEID = %u\r\n", LOCAL_ADDR);

	// radio channel
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "CHANNEL = %u", &channel) != 1 || channel < 11 || channel > 26) {
		PRINTF("Error reading radio channel\r\n");
		sdready = false;
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	RADIO_CHANNEL = (uint8_t)channel;
	PRINTF("CHANNEL = %u\r\n", RADIO_CHANNEL);

	// radio power
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "POWER = %u", &power) != 1 || power > 0xF) {
		sdready = false;
		PRINTF("Error reading radio power\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	RADIO_POWER = (uint8_t)power;
	PRINTF("POWER = %u\r\n", RADIO_POWER);

	// sensing mechanism
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TRIG = %u", &idx) != 1 || idx > 1) {
		sdready = false;
		PRINTF("Error reading sensing mechanism\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	TrigSenIdx = (uint8_t)idx;
	PRINTF("TRIG = %u\r\n", TrigSenIdx);
	f_close(&file);
	// update NAND Flash if needed
	fc.nodeid = LOCAL_ADDR;
	fc.channel = RADIO_CHANNEL;
	fc.power = RADIO_POWER;
	fc.trigsen = TrigSenIdx;
	if (memcmp(&fc, fcptr, sizeof (flashconfig_t)) != 0) {
		PRINTF("Updating NAND Flash...\r\n");
		memcpy(flashbuf, &fc, sizeof (flashconfig_t));
		if (NandFlash_BlockErase(NANDFLASH_BLOCK_XNODECFG) != NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash erase failed\r\n");
		} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
		0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash write success\r\n");
		} else {
			PRINTF("NAND Flash write failed\r\n");
		}
	}

	sdfree(flashbuf);

	return SUCCESS;
}

int SDCard_Reset(void) // should be called NAND_reset
{
	uint32_t i;
	NandFlash_Init();

	for (i=0;i<1024;i++){
		PRINTF("DETELE: %d\n\r",i);
		if (NandFlash_BlockErase(i) != NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash block %d erase failed\r\n",i);
		}
	}

	return SUCCESS;
}

int SDCard_ReWrite(int trignum) // no init
{
	flashconfig_t fc;
	uint8_t *flashbuf;

	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// update NAND Flash if needed
	fc.nodeid = LOCAL_ADDR;
	fc.channel = RADIO_CHANNEL;
	fc.power = RADIO_POWER;
	fc.trigsen = trignum;
	PRINTF("Updating NAND Flash...\r\n");
	memcpy(flashbuf, &fc, sizeof (flashconfig_t));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_XNODECFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}

	sdfree(flashbuf);
	return SUCCESS;
}


int SDCard_ReWriteALL(int nid, int chl, int pwr, int tix) // no init
{
	flashconfig_t fc;
	uint8_t *flashbuf;

	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// update NAND Flash if needed
	fc.nodeid = nid;
	fc.channel = chl;
	fc.power = pwr;
	fc.trigsen = tix;
	PRINTF("Updating NAND Flash...\r\n");
	memcpy(flashbuf, &fc, sizeof (flashconfig_t));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_XNODECFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}

	sdfree(flashbuf);
	return SUCCESS;
}

#else

#include <4GFTP.h>

extern uint16_t rsnodes[MAX_NODES];
extern uint8_t rsncnt, rschannels;
extern uint16_t rstime, rsrate;
extern bool issendonly;
extern int adxl_rate;
extern uint16_t adxl_range;
extern uint16_t thres_act; // set activity threshold detection as 200mg (if range is 2g, LSB/g is 1000)
extern uint16_t time_act; // set time for activity detection as 50ms (if ODR is 100, time is 5/100s = 50ms)
extern uint16_t thres_iact; // set activity threshold detection as 200mg (if range is 2g, LSB/g is 1000)
extern uint16_t time_iact; // set time for activity detection as 50ms (if ODR is 100, time is 5/100s = 50ms)
extern uint16_t FIFO_numb;
extern uint8_t task1T;
extern uint8_t task2T;
extern uint8_t task3T;
uint64_t pnum;
uint16_t thres_actOriginal, thres_iactOriginal;
uint16_t blimit, sllimit, sulimit, ftps1, ftps2, ftps3, ftps4;
uint64_t pnumber;
char ftpun[30], ftpp[30], ftpa[30];
uint8_t  channel_4G;

int SDCard_Init(void)
{
	FRESULT ret;              // Result code
  FIL file;                 // File object
	char line[LINELEN];
	uint16_t nodeid, node;
	uint8_t  channel, power;
	int res = FAIL;
	flashconfig_t fc, *fcptr;
  uint8_t *flashbuf;
	char *nline, *str = 0;
  uint32_t i;
	
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	
	// safe values, if things don't work out for both nand and sd, use these number
	fc.power = 0;
	fc.task1time = 5;
	fc.task2time = 1;
	fc.task3time = 12;
	fc.adxlrange = 8;
	fc.adxlthresholdact = 200;
	fc.adxltimeact = 2;
	fc.adxlthresholdinact = 80;
	fc.adxltimeinact = 500;
	fc.adxlfifonum = 60;
	fc.remotesensingtime = 180;
	fc.remotesensingrate = 100;
	fc.blocklimit = 500;
	fc.senselowerlimit = 5;
	fc.senseupperlimit = 400;
	fc.phonenumber = 7145487654;
	fc.ftpsite1 = 34;      
	fc.ftpsite2 = 209;
	fc.ftpsite3 = 83;
	fc.ftpsite4 = 169;
	memcpy(&fc.ftpusername,"1112223333@bell.ca",18);
	memcpy(&fc.ftppassword,"12345678",8);
	memcpy(&fc.ftpapn,"crstat.bell.ca",14);
	
	// try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
  NandFlash_Init();
  if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
  0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
    PRINTF("NAND Flash read success\r\n");
		fcptr = (flashconfig_t *)flashbuf;
		LOCAL_ADDR = fcptr->nodeid;
		RADIO_CHANNEL = fcptr->channel;
		RADIO_POWER = fcptr->power;
		task1T = fcptr->task1time;
		task2T = fcptr->task2time;
		task3T = fcptr->task3time;
		adxl_range = fcptr->adxlrange;
		thres_actOriginal = fcptr->adxlthresholdact;
		time_act = fcptr->adxltimeact;
		thres_iactOriginal = fcptr->adxlthresholdinact;
		time_iact = fcptr->adxltimeinact;
		FIFO_numb = fcptr->adxlfifonum;
		rstime = fcptr->remotesensingtime;
		rsrate = fcptr->remotesensingrate;
		rschannels = fcptr->remotesensingchannels;
		rsncnt = fcptr->remotesensingncnt;
		blimit = fcptr->blocklimit;
		sllimit = fcptr->senselowerlimit;
		sulimit = fcptr->senseupperlimit;
		pnumber = fcptr->phonenumber;
		ftps1 = fcptr->ftpsite1;
		ftps2 = fcptr->ftpsite2;
		ftps3 = fcptr->ftpsite3;
		ftps4 = fcptr->ftpsite4;
		memcpy(&ftpun,&fcptr->ftpusername,strnlen((char *)&fcptr->ftpusername, 30));
		memcpy(&ftpp,&fcptr->ftppassword,strnlen((char *)&fcptr->ftppassword, 30));
		memcpy(&ftpa,&fcptr->ftpapn,strnlen((char *)&fcptr->ftpapn, 30));
		if ((rsncnt>0) && (rsncnt<30)){ // prevent infinite loop
			for (i=0;i<rsncnt;i++){
				rsnodes[i] = fcptr->remotesensingnodes[i];
			}
		}
		res = SUCCESS;
	} else {
    PRINTF("NAND Flash read failed\r\n");
		res = FAIL;
  }
	
	PRINTF("LOCAL_ADDR = %d\n\r",LOCAL_ADDR);
	PRINTF("RADIO_CHANNEL = %d\n\r",RADIO_CHANNEL);
	PRINTF("RADIO_POWER = %d\n\r",RADIO_POWER);
	PRINTF("task1T = %d\n\r",task1T);
	PRINTF("task2T = %d\n\r",task2T);
	PRINTF("task3T = %d\n\r",task3T);
	PRINTF("adxl_range = %d\n\r",adxl_range);
	PRINTF("thres_actOriginal = %d\n\r",thres_actOriginal);
	PRINTF("time_act = %d\n\r",time_act);
	PRINTF("thres_iactOriginal = %d\n\r",thres_iactOriginal);
	PRINTF("time_iact = %d\n\r",time_iact);
	PRINTF("FIFO_numb = %d\n\r",FIFO_numb);
	PRINTF("time = %d\n\r",rstime);
	PRINTF("rate = %d\n\r",rsrate);
	PRINTF("channels = %d\n\r",rschannels);
	PRINTF("ncnt = %d\n\r",rsncnt);
	PRINTF("nodes = ");

	if ((rsncnt>0) && (rsncnt<30)){ // prevent infinite loop
		for (i=0;i<rsncnt;i++){
			PRINTF("%d    ",rsnodes[i]);
		}
	PRINTF("\n\r");
	PRINTF("block_limit = %d\n\r", blimit);
	PRINTF("sense_lowerlimit = %d\n\r", sllimit);
	PRINTF("sense_upperlimit = %d\n\r", sulimit);
	}
	switch (adxl_range)
	{
		case 2: adxl_rate=1; break;
		case 4: adxl_rate=2; break;
		case 8: adxl_rate=4; break;
		default: PRINTF("Error value of range\r\n"); break;
	}
	thres_act = (uint16_t)thres_actOriginal/adxl_rate;
	thres_iact = (uint16_t)thres_iactOriginal/adxl_rate;
	PRINTF("thres_act = %d\n\r",thres_act);
	PRINTF("thres_iact = %d\n\r",thres_iact);
	
	// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);

  // go to config dir	
	ret = f_chdir("/Xnode");
  if (ret) {
    PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		sdfree(flashbuf);
		if (ret == FR_NOT_READY)
		{
			SD_ISIN = 0;
		}
		return res;
	}

	// read config file
  ret = f_open(&file, "Xnode.cfg", FA_READ);
  if (ret) {
    PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		sdfree(flashbuf);
		return res;
	}

	// node id
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "NODEID = %hu", &nodeid) != 1 || nodeid >= 0xFFFF) {
    PRINTF("Error reading node id\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	LOCAL_ADDR = (uint16_t)nodeid;
	PRINTF("NODEID = %u\r\n", LOCAL_ADDR);
	
	// radio channel
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "CHANNEL = %hhu", &channel) != 1 || channel < 11 || channel > 26) {
    PRINTF("Error reading radio channel\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	channel_4G = (uint8_t)channel;
	RADIO_CHANNEL = (uint8_t)channel;
	PRINTF("CHANNEL = %u\r\n", RADIO_CHANNEL);

	// radio power
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "POWER = %hhu", &power) != 1 || power > 0xF) {
    PRINTF("Error reading radio power\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	RADIO_POWER = (uint8_t)power;
	PRINTF("POWER = %u\r\n", RADIO_POWER);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TASK1_TIME = %hhu", &task1T) != 1 || task1T > 0xF) {
		PRINTF("Error reading radio power\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	PRINTF("TASK1_TIME = %u\r\n", task1T);

	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TASK2_TIME = %hhu", &task2T) != 1 || task2T > 0xF) {
		PRINTF("Error reading radio power\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	PRINTF("TASK2_TIME = %u\r\n", task2T);

	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TASK3_TIME = %hhu", &task3T) != 1 || task3T > 0xF) {
		PRINTF("Error reading radio power\r\n");
		f_close(&file);
		sdfree(flashbuf);
		return res;
	}
	PRINTF("TASK3_TIME = %u\r\n", task3T);
	
	//ADXL
	// measurement range
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "RANGE = %hu", &adxl_range) != 1 || adxl_range > 8) {
    PRINTF("Error reading  measurement range\r\n");
		f_close(&file);
		return FAIL;
	}
	PRINTF("RANGE = %u\r\n", adxl_range);
	switch (adxl_range)
	{
		case 2: adxl_rate=1; break;
		case 4: adxl_rate=2; break;
		case 8: adxl_rate=4; break;
		default: PRINTF("Error value of range\r\n"); break;
	}
	PRINTF("adxl_rate is %u\r\n", adxl_rate);

	
	// threshold for activity detection
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "THRESHOLDACT = %hu", &thres_actOriginal) != 1 || thres_actOriginal >= 0xFFFF) {
    PRINTF("Error reading threshold for activity detection\r\n");
		f_close(&file);
		return res;
	}
	thres_act = (uint16_t)thres_actOriginal/adxl_rate;
	PRINTF("THRESHOLDACT = %u\r\n", thres_act);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TIMEACT = %hu", &time_act) != 1 || time_act >= 0xFFFF) {
    PRINTF("Error reading time for activity detection\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("TIMEACT = %u\r\n", time_act);
	
	// threshold for inactivity detection
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "THRESHOLDINACT = %hu", &thres_iactOriginal) != 1 || thres_iactOriginal >= 0xFFFF) {
    PRINTF("Error reading threshold for inactivity detection\r\n");
		f_close(&file);
		return res;
	}
	thres_iact = (uint16_t)thres_iactOriginal/adxl_rate;
	PRINTF("THRESHOLDINACT = %u\r\n", thres_iact);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TIMEINACT = %hu", &time_iact) != 1 || time_iact >= 0xFFFF) {
    PRINTF("Error reading time for activity detection\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("TIMEINACT = %u\r\n", time_iact);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "FIFONUM = %hu", &FIFO_numb) != 1 || FIFO_numb >= 0xFFFF) {
    PRINTF("Error reading time for activity detection\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("FIFONUM = %u\r\n", FIFO_numb);

	// sampling time
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SAMPLINGTIME = %hu", &rstime) != 1 || rstime > 0xFFFF) {
    PRINTF("Error reading sampling time\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("SAMPLINGTIME = %u\r\n", rstime);

	// sampling rate
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SAMPLINGRATE = %hu", &rsrate) != 1 || rsrate >= 0xFFFF) {
    PRINTF("Error reading sampling rate\r\n");
		f_close(&file);
		return res;
	}
	/* kam */
#ifdef STRAIN
#ifdef KHZ
	rsrate = 1000;
#else
	rsrate = 500;
#endif
#endif
	PRINTF("SAMPLINGRATE = %u\r\n", rsrate);

	// number of channels
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "NUMCHANNELS = %hhu", &rschannels) != 1 || rschannels < 1 || rschannels > 8) {
    PRINTF("Error reading number of channels\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("NUMCHANNELS = %u\r\n", rschannels);

	// number of nodes
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "NUMNODES = %hhu", &rsncnt) != 1 || rsncnt > MAX_NODES) {
    PRINTF("Error reading number of nodes\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("NUMNODES = %u\r\n", rsncnt);
	
	// node ids
	nline = sdmalloc(512);
	configASSERT(nline);
	if (f_gets(nline, 512, &file) == NULL) {
    PRINTF("Error reading node ids\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("%s\r", nline);

	// parse nodeids
	str = strchr(nline, '=');
	if (str == NULL) {
		f_close(&file);
		return res;
	}
	str++;
	str = strtok(str, " ,\r\n");
	for (i = 0; i < rsncnt && str != NULL; ++i, str = strtok(NULL, " ,\r\n")) {
		if (sscanf(str, "%hu", &node) != 1 || node < 1 || node >= 0xFFFF) {
			break;
		}
		rsnodes[i] = (uint16_t)node;
	}
	sdfree(nline);
	
	// Limit pages for NAND before erasing
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "BLOCKLIMIT = %hu", &blimit) != 1 || blimit > NANDFLASH_NUMOF_BLOCK) {
    PRINTF("Error reading limit block number\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("BLOCKLIMIT = %u\r\n", blimit);	
	
	// Limit sensing
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SENSELOWERLIMIT = %hu", &sllimit) != 1 || sllimit > 1200) {
    PRINTF("Error reading sensing lower limit\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("SENSELOWERLIMIT = %u\r\n", sllimit);	

	// Limit sensing
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SENSEUPPERLIMIT = %hu", &sulimit) != 1 || sulimit > 1200) {
    PRINTF("Error reading sensing upper limit\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("SENSEUPPERLIMIT = %u\r\n", sulimit);	
	
	// Phone number
	// if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "PHONENUMBER = %llu", &pnumber) != 1 || pnumber < 1000000000 || pnumber > 9999999999) {
    // PRINTF("Error reading phone number\r\n");
	// 	f_close(&file);
	// 	return res;
	// }
	// PRINTF("PHONENUMBER = %llu\r\n", pnumber);	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "PHONENUMBER = %llu", &pnumber) != 1) {
    PRINTF("Error reading phone number\r\n");
		f_close(&file);
		return res;
	}
	PRINTF("PHONENUMBER = %llu\r\n", pnumber);

	// ftp site 1
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SITE1 = %hu", &ftps1) != 1) {
    PRINTF("Error reading ftp 1\r\n");
		f_close(&file);
		return res;
	}
	
	// ftp site 2
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SITE2 = %hu", &ftps2) != 1) {
    PRINTF("Error reading ftp 2\r\n");
		f_close(&file);
		return res;
	}

	// ftp site 3
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SITE3 = %hu", &ftps3) != 1) {
    PRINTF("Error reading ftp 3\r\n");
		f_close(&file);
		return res;
	}

	// ftp site 4
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SITE4 = %hu", &ftps4) != 1) {
    PRINTF("Error reading ftp 4\r\n");
		f_close(&file);
		return res;
	}	
	PRINTF("Site = %hu.%hu.%hu.%hu\r\n", ftps1, ftps2, ftps3, ftps4);	

	// ftp username
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "USERNAME = %s", ftpun) != 1) {
    PRINTF("Error reading ftp username\r\n");
		f_close(&file);
		return res;
	}	
	PRINTF("USERNAME = %s\r\n", ftpun);	

	// ftp password
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "PASSWORD = %s", ftpp) != 1) {
    PRINTF("Error reading ftp password\r\n");
		f_close(&file);
		return res;
	}	
	PRINTF("PASSWORD = %s\r\n", ftpp);	
	
	// ftp apn
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "APN = %s", ftpa) != 1) {
    PRINTF("Error reading ftp apn\r\n");
		f_close(&file);
		return res;
	}	
	PRINTF("APN = %s\r\n", ftpa);

  f_close(&file);
	
	// update NAND Flash if needed
  fc.nodeid = LOCAL_ADDR;
	fc.channel = RADIO_CHANNEL;
	fc.power = RADIO_POWER;
	fc.task1time = task1T;
	fc.task2time = task2T;
	fc.task3time = task3T;
	fc.adxlrange = adxl_range;
	fc.adxlthresholdact = thres_actOriginal;
	fc.adxltimeact = time_act;
	fc.adxlthresholdinact = thres_iactOriginal;
	fc.adxltimeinact = time_iact;
	fc.adxlfifonum = FIFO_numb;
	fc.remotesensingtime = rstime;
	fc.remotesensingrate = rsrate;
	fc.remotesensingchannels = rschannels;
	fc.remotesensingncnt = rsncnt;
	fc.blocklimit = blimit;	//TUADD: Limit of # of dirty block before erasing
	fc.senselowerlimit = sllimit;	//TUADD: Limit of lower bound for sensing (ms)
	fc.senseupperlimit = sulimit;	//TUADD: Limit of upper bound for sensing (ms)
	fc.phonenumber = pnumber;	//TUADD: phone number for 4G
	fc.ftpsite1 = ftps1;	//TUADD: ftp 1
	fc.ftpsite2 = ftps2;	//TUADD: ftp 2
	fc.ftpsite3 = ftps3;	//TUADD: ftp 3
	fc.ftpsite4 = ftps4;	//TUADD: ftp 4
	memcpy(&fc.ftpusername, &ftpun, strnlen(ftpun, 30));
	memcpy(&fc.ftppassword, &ftpp, strnlen(ftpp, 30));
	memcpy(&fc.ftpapn, &ftpa, strnlen(ftpa, 30));
	for (i=0;i<rsncnt;i++){
		fc.remotesensingnodes[i] = rsnodes[i];
	}
	if (memcmp(&fc, fcptr, sizeof (flashconfig_t)) != 0) {
		PRINTF("Updating NAND Flash...\r\n");
		memcpy(flashbuf, &fc, sizeof (flashconfig_t));
		if (NandFlash_BlockErase(NANDFLASH_BLOCK_XNODECFG) != NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash erase failed\r\n");
		} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
		0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash write success\r\n");
		} else {
			PRINTF("NAND Flash write failed\r\n");
		}
	}
	sdfree(flashbuf);
	return SUCCESS;
}

int write_tosend(bool istosend) // if there's something to send, write 1, else write 0
{
	uint8_t *flashbuf;
		// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	
	memcpy(flashbuf, &istosend, sizeof (istosend));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_SENSOR_TOSENDCFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_SENSOR_TOSENDCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}
	
	PRINTF("Finished writing TOSEND = %d\n\r",istosend);
	return SUCCESS;
}

int read_tosend() // if there's something to send, output 1, else output 0
{
	uint8_t istosend;
  uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

 // try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
  if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_SENSOR_TOSENDCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
  0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
    PRINTF("NAND Flash read success\r\n");
		istosend = (bool) *flashbuf;
	} else {
    PRINTF("NAND Flash read failed\r\n");
  }
	PRINTF("istosend = %d\n\r",istosend);
	
		// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	// write to NAND flash
	if (istosend != (bool)flashbuf)
	{
		memcpy(flashbuf, &istosend, sizeof (istosend));
		if (NandFlash_BlockErase(NANDFLASH_BLOCK_SENSOR_TOSENDCFG) != NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash erase failed\r\n");
		} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_SENSOR_TOSENDCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
		0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash write success\r\n");
		} else {
			PRINTF("NAND Flash write failed\r\n");
		}	
	}	
	sdfree(flashbuf);	

	return istosend;
}

int SDCard_Reset(void) // should be called NAND_reset
{
  uint8_t *flashbuf;
  uint32_t i;
	uint32_t datatowrite;
	
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// reset data counter
	datatowrite = 0;
	memcpy(flashbuf, &datatowrite, sizeof (datatowrite));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_DATACFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_DATACFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}	
	
	// reset data left (to be sent on gateway) counter
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}		

	// reset if sensor node needs to send or not
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_SENSOR_TOSENDCFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_SENSOR_TOSENDCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}		

	// erase saved alarm time
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_UNIXALARMTIME) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	}	
	
	// erase all data
	for (i=0;i<NANDFLASH_BLOCK_DATALIMIT;i++){
		if (NandFlash_BlockErase(i) != NANDFLASH_STATUS_SUCCESS) {
			lpc_printf("NAND Flash block %d erase failed\r\n",i);
		}
	}
		
	return SUCCESS;
}

int SDCard_ReWrite(int trignum) // no init
{
	flashconfig_t fc;
	uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// update NAND Flash if needed
	fc.nodeid = LOCAL_ADDR;
	fc.channel = RADIO_CHANNEL;
	fc.power = RADIO_POWER;
	//fc.trigsen = trignum; // Do we need trigsen = 0 for FRA in any case? TUFIXDONE. Fix: We might, BUT it could interfere with train running by
	PRINTF("Updating NAND Flash...\r\n");
	memcpy(flashbuf, &fc, sizeof (flashconfig_t));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_XNODECFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}

	sdfree(flashbuf);
	return SUCCESS;
}


int SDCard_ReWriteALL(int nid, int chl, int pwr, int tix) // no init
{
	flashconfig_t fc;
	uint8_t *flashbuf;

	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// update NAND Flash if needed
	fc.nodeid = nid;
	fc.channel = chl;
	fc.power = pwr;
	//fc.trigsen = tix; // Do we need trigsen = 0 for FRA in any case? TUFIXDONE. Fix: We might, BUT it could interfere with train running by
	PRINTF("Updating NAND Flash...\r\n");
	memcpy(flashbuf, &fc, sizeof (flashconfig_t));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_XNODECFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}

	sdfree(flashbuf);
	return SUCCESS;
}

void write_nand2sdattemp(bool nand2sdattemp)
{
	uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	
	PRINTF("Write NAND2SD attempt: %d\r\n", nand2sdattemp);
	flashbuf[0] = nand2sdattemp;
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_NAND2SDATTEMP) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_NAND2SDATTEMP * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}
}

bool read_nand2sdattemp(void)
{
	uint8_t *flashbuf;
	bool nand2sdattemp;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	
	PRINTF("Reading NAND Flash...\r\n");
  if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_NAND2SDATTEMP * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
  0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
    PRINTF("NAND Flash read success\r\n");
		nand2sdattemp = (bool) flashbuf[0];
	} else {
    PRINTF("NAND Flash read failed\r\n");
  }
	PRINTF("Read NAND2SD attempt: %d\n\r", nand2sdattemp);
	return nand2sdattemp;
}

int NAND2SD()
{
#if 0
	// before this, make sure all the data are used for its purpose (sent out to gateway, upload to ftp)
	uint8_t *flashbuf;
	FRESULT ret;              // Result code
  FIL file;     
	int i,j;
	uint32_t dlen = 0,ptr = 0; // length of buffer and pointer to current writing position
	uint8_t year, month, date, hour, minute, second;
	char line_write[121];
	UINT len_fwrite;
	
	// at the earliest, write 1 to block NANDFLASH_BLOCK_NAND2SDATTEMP, and write 0 once done, so if it fails (crash, bad card, not enough energy), we know we shouldn't try nand2sd again
	write_nand2sdattemp(true);
#if 0	// Some NAND are not good, and reading multiple pages got stuck
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	// create file in sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	PRINTF("Writing raw binary data to sdcard...");
	ret = f_chdir("/Data");
	if (ret) {
    lpc_printf("FAIL\r\n");
	}

	// write data file
	gettime(&year, &month, &date, &hour, &minute, &second);
	snprintf(line_write, LINELEN, "NAND2SD_%03u_%02u%02u%02u_%02u%02u%02u.txt", LOCAL_ADDR,year,month,date,hour,minute,second);	// nodeid_yyddmm_hhmmss_index
  ret = f_open(&file, line_write, FA_WRITE | FA_CREATE_ALWAYS);
  if (ret) {
    lpc_printf("Create a new file error\n");
	}
	// iterate from block 1 to block  (here use 901)
		// read	all pages from block (128kb) and write to sd file
		// erase block
	for  (i=1;i<NANDFLASH_BLOCK_DATALIMIT+1;i++) // TUFIXDONE: 900 is the # blocks for NAND, look into this and make sure it's the same as the 500 blocks limit - different, but it was correct
	{
		if (i%100 == 1)
		{
			PRINTF("page: %d\n\r",i);
		}
		for (j=0;j<64;j++)
		{
			if (NandFlash_PageRead(flashbuf, i*64+j, 0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
				//lpc_printf("NAND Flash read success\r\n");
			} else {
				lpc_printf("NAND Flash read failed\r\n");
			}
		// write buffer to file	
			len_fwrite = 0;
			dlen = NANDFLASH_RW_PAGE_SIZE;
			f_lseek(&file, ptr);
			do
			{
				dlen = dlen - len_fwrite;
				f_write(&file, flashbuf + len_fwrite, dlen, &len_fwrite);
				//lpc_printf("len_fwrite = %d, ptr = %d\n\r",len_fwrite, ptr);
			} while (len_fwrite > 0);
			ptr = ptr + NANDFLASH_RW_PAGE_SIZE; // increase  the pointer by length of just written data buffer
		}
	}
	f_close(&file);
	PRINTF("Done\r\n");
#endif	
	write_nand2sdattemp(false);
#endif
	return SUCCESS;
}

#ifdef FRA
int SDCard_ReWriteCN(flashconfig_t fc) // no init
{
	// MUST HAVE A HARD-CODED VERSION OF THE XNODE IN HERE IN CASE PARAMETERS ARE BROKEN
	flashconfig_t *fcptr;
	uint8_t *flashbuf, len = 0;
	const char *sdname[28];
	uint8_t i,j;
	bool sdfail = false;
	FRESULT ret;              
	FIL file;
	char line[80];
	uint8_t similarcount = 0;
#ifdef GATEWAY	
	const uint8_t similarCONST = 11;
#else
	const uint8_t similarCONST = 6;
#endif
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);

	// try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
	NandFlash_Init();
	if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash read success\r\n");
		fcptr = (flashconfig_t *)flashbuf;
	} else {
		PRINTF("NAND Flash read failed\r\n");
	}
	
// Copy parameters that are not set by the server
		if (fc.adxlrange == 0) { fc.adxlrange=fcptr->adxlrange;}
		if (fc.adxlthresholdact == 0) { fc.adxlthresholdact=fcptr->adxlthresholdact;}
		if (fc.adxltimeact == 0) { fc.adxltimeact=fcptr->adxltimeact;}
		if (fc.adxlthresholdinact == 0) { fc.adxlthresholdinact=fcptr->adxlthresholdinact;}
		if (fc.adxltimeinact == 0) { fc.adxltimeinact=fcptr->adxltimeinact;}
		if (fc.task1time == 0) { fc.task1time=fcptr->task1time;}
		if (fc.task2time == 0) { fc.task2time=fcptr->task2time;}
		if (fc.task3time == 0) { fc.task3time=fcptr->task3time;}
		if (fc.senselowerlimit == 0) { fc.senselowerlimit=fcptr->senselowerlimit;}
		if (fc.senseupperlimit == 0) { fc.senseupperlimit=fcptr->senseupperlimit;}
		if (fc.phonenumber == 0) { fc.phonenumber=fcptr->phonenumber;}
		
 // only check the part we care, leave the other parameters untouched
		((fc.adxlrange == fcptr->adxlrange) || (fc.adxlrange != 2) || (fc.adxlrange != 4) || (fc.adxlrange != 8)) ? (similarcount += 1): (fcptr->adxlrange = fc.adxlrange); // Sanity check: adxl range, must be 2/4/8 (G)
		((fc.adxlthresholdact == fcptr->adxlthresholdact) || (fc.adxlthresholdact > fc.adxlrange*1000) || (fc.adxlthresholdact <= 20)) ? (similarcount += 1): (fcptr->adxlthresholdact = fc.adxlthresholdact); // Sanity check: adxl activity detection, must < range (in mg), and must be at least 80mg so that the sensor is not triggered all the time
		((fc.adxltimeact == fcptr->adxltimeact) || (fc.adxltimeact > 5))? (similarcount += 1): (fcptr->adxltimeact = fc.adxltimeact); // Sanity check: adxl over-threshold points
		((fc.adxlthresholdinact == fcptr->adxlthresholdinact) || (fc.adxlthresholdinact > fc.adxlrange*1000) || (fc.adxlthresholdinact < 10))? (similarcount += 1): (fcptr->adxlthresholdinact = fc.adxlthresholdinact); // Sanity check: inactivity detection, must < activity threshold (mg),, and must be at least 40mg so that the sensor is not triggered all the time
		((fc.adxltimeinact == fcptr->adxltimeinact) || (fc.adxltimeinact > 500) || (fc.adxltimeinact < 100)) ? (similarcount += 1): (fcptr->adxltimeinact = fc.adxltimeinact); // Sanity check: inactivity time (in ms), too short will lead to no data recorded
		((fc.task1time == fcptr->task1time) || (fc.task1time >= 60) || (60%fc.task1time != 0)|| (fc.task1time < 2))  ? (similarcount += 1) : (fcptr->task1time = fc.task1time); // Sanity check: every # minutes gateway wakes up and retrieve data, must < 60
#ifdef GATEWAY		// the 1st part is only applicable for sensor nodes, can modify this part for future usess// interesting, without the last 2 () of the ternary operator, this will be wrong due to the way the expression is read
		((fc.task2time == fcptr->task2time) || (fc.task2time > 48)) ? (similarcount += 1) : (fcptr->task2time = fc.task2time); // Sanity check: every # hours gateway wakes up and get status, can be up to 2 days
		((fc.task3time == fcptr->task3time) || ((fc.task3time%2 != 0) && (fc.task3time%3 != 0)) || (fc.task3time > 24)) ? (similarcount += 1) : (fcptr->task3time = fc.task3time); // Sanity check: every # hours gateway wakes up to update config paratemeters, for now can't be more than 1 day
		((fc.senselowerlimit == fcptr->senselowerlimit) || (fc.senselowerlimit < 5) || (fc.senselowerlimit > 10)) ? (similarcount += 1): (fcptr->senselowerlimit = fc.senselowerlimit); // Sanity check: lower limit to ignore data (in seconds). CN app WILL NOT use this number
		((fc.senseupperlimit == fcptr->senseupperlimit) || (fc.senseupperlimit > 400) || (fc.senseupperlimit < fc.senselowerlimit)) ? (similarcount += 1): (fcptr->senseupperlimit = fc.senseupperlimit); // Sanity check: upper limit to stop sensing, 400 is the limit now. TUFIX: Check higher limits 
		(fc.phonenumber == fcptr->phonenumber) ? (similarcount += 1): (fcptr->phonenumber = fc.phonenumber); // Sanity check: This won't be broken, can change again
#endif
	if (similarcount != similarCONST) // only rerwite of changes are found. TUFIX: mempcy the 2 structs doesn't  work
	{
		PRINTF("Updating NAND Flash...\r\n");
		if (NandFlash_BlockErase(NANDFLASH_BLOCK_XNODECFG) != NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash erase failed\r\n");
		} else if (NandFlash_PageProgram((uint8_t*) fcptr, NANDFLASH_BLOCK_XNODECFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
		0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash write success\r\n");
		} else {
			PRINTF("NAND Flash write failed\r\n");
		}
	
		sdfree(flashbuf);

		// SD
		sdname[0] = "NODEID";
		sdname[1] = "CHANNEL";
		sdname[2] = "POWER";
		sdname[3] = "TASK1_TIME";
		sdname[4] = "TASK2_TIME";	
		sdname[5] = "TASK3_TIME";		
		sdname[6] = "RANGE";
		sdname[7] = "THRESHOLDACT";
		sdname[8] = "TIMEACT";
		sdname[9] = "THRESHOLDINACT";
		sdname[10] = "TIMEINACT";
		sdname[11] = "FIFONUM";
		sdname[12] = "SAMPLINGTIME";
		sdname[13] = "SAMPLINGRATE";
		sdname[14] = "NUMCHANNELS";
		sdname[15] = "NUMNODES";
		sdname[16] = "NODEIDS";
		sdname[17] = "BLOCKLIMIT";
		sdname[18] = "SENSELOWERLIMIT";
		sdname[19] = "SENSEUPPERLIMIT";
		sdname[20] = "PHONENUMBER";
		sdname[21] = "SITE1";
		sdname[22] = "SITE2";
		sdname[23] = "SITE3";
		sdname[24] = "SITE4";
		sdname[25] = "USERNAME";
		sdname[26] = "PASSWORD";
		sdname[27] = "APN";
		
		// mount sdcard
		memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
		f_mount(0, &Fatfs);

		// go to data dir	
		ret = f_chdir("/Xnode");	
		
		if (ret) {
			PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
			sdfail = true;
			//return res; // move on
		} 
			// read config file
			ret = f_open(&file, "Xnode.cfg", FA_WRITE);
			if (ret) {
				PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
				sdfail = true;
				//return res; // move on
		}
		memcpy(&ftpun,&fcptr->ftpusername,strnlen((char *)&fcptr->ftpusername,30)); // TUFIX: These fixed length are not the best options, but stick to it for now
		memcpy(&ftpp,&fcptr->ftppassword,strnlen((char *)&fcptr->ftppassword,30)); // TUFIX: These fixed length are not the best options, but stick to it for now
		memcpy(&ftpp,&fcptr->ftpapn,strnlen((char *)&fcptr->ftpapn,30)); // TUFIX: These fixed length are not the best options, but stick to it for now
		// write data file
		if (!sdfail)
		{
			for (i = 0; i < 28; i++)
			{
				switch (i) {
				case 0: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->nodeid); break;
				case 1: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->channel); break;
				case 2: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->power); break;
				case 3: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->task1time); break;
				case 4: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->task2time); break;
				case 5: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->task3time); break;				
				case 6: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->adxlrange); break;
				case 7: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->adxlthresholdact); break;
				case 8: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->adxltimeact); break;
				case 9: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->adxlthresholdinact); break;
				case 10: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->adxltimeinact); break;
				case 11: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->adxlfifonum); break;
				case 12: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->remotesensingtime); break;			
				case 13: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->remotesensingrate); break;
				case 14: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->remotesensingchannels); break;
				case 15: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->remotesensingncnt); break;
				case 16: 
					len = 0;
					len = len + snprintf(line+len, LINELEN, "%s = %u",sdname[i], fcptr->remotesensingnodes[0]); 
					for (j = 1; j < fcptr->remotesensingncnt; j++)
					{
						len = len + snprintf(line+len, LINELEN, ", %u",fcptr->remotesensingnodes[j]); 
					}
					len = len + snprintf(line+len, LINELEN, "\n"); 
					break;
				case 17: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->blocklimit); break;
				case 18: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->senselowerlimit); break;
				case 19: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->senseupperlimit); break;
				case 20: snprintf(line, LINELEN, "%s = %llu\n",sdname[i], fcptr->phonenumber); break;
				case 21: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->ftpsite1); break;
				case 22: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->ftpsite2); break;
				case 23: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->ftpsite3); break;
				case 24: snprintf(line, LINELEN, "%s = %u\n",sdname[i], fcptr->ftpsite4); break;		
				case 25: snprintf(line, LINELEN, "%s = %s\n",sdname[i], ftpun); break;
				case 26: snprintf(line, LINELEN, "%s = %s\n",sdname[i], ftpp); break;				
				case 27: snprintf(line, LINELEN, "%s = %s\n",sdname[i], ftpa); break;			
				}
				lpc_printf("WRITE: %s\r",line);
				if (f_puts(line, &file) == 0) {
					PRINTF("Error writing data\r\n");
					f_close(&file);
					break;
				}
			}
		}
		f_close(&file);
	}
	return SUCCESS;
	
}
#endif
#endif

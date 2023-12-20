#include <xnode.h>
#include <timers.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <ff.h>
#include <ads131.h>
#include <pmic.h>
#include <radio.h>
#include <ReliableComm.h>
#include <RemoteCommand.h>
#include <Sensing.h>
#include <Utils.h>
#include "RemoteSensing.h"
#include <math.h>
#include <lpc43xx_rit.h>
#include "triggersensing.h"
#include "rtc.h"
#include "timesync.h"
#include "GlobalTime.h"
#include "yaffsHEADER.h"
#include <SnoozeAlarm.h>
#include <nandflash_k9f1g08u0a.h> 
#include <sdcard.h>
#include <4GFTP.h>

#define LINELEN               120

typedef struct {
	uint32_t sampleTime;
	uint16_t samplingRate;
	uint16_t dataRate;
	uint64_t time64;
	uint8_t channelMask;
	chType ctype;
} __attribute__((packed)) sumessage;

uint16_t rsnodes[MAX_NODES];
uint8_t rsncnt, rschannels;
uint16_t rstime, rsrate;
uint32_t data_index;
TimerHandle_t tim;

static sumessage sumsg;
static bool initialized = false, dataready = false, spsuccess;
#ifndef FRA
uint8_t rsidx, rsok, rcid = 0xff;
#else
uint8_t rsok, rcid = 0xff;
uint32_t rsidx;
#endif
static SensorData *sd;
TaskHandle_t xTaskToNotify = NULL;
static TimerHandle_t tim_sn = NULL;
static float *vcinfo;//[2 * MAX_NODES];
extern bool sdready;
static int len;
static char line_write[121];
extern uint8_t TrigSenIdx;
extern bool isformat;
extern uint32_t dslen;

static void print_vcinfo(void)
{
	int i;
	lpc_printf("- Voltage and Charging Current for responsive nodes:\r\n");
	lpc_printf("- Node\tVbat (V)\tIchg (mA)\r\n");
	for (i = 0; i < rsncnt; ++i) {
		if (vcinfo[2*i] > 0.f) {
			lpc_printf("- %03u\t%.2f\t\t%.2fmA\r\n", rsnodes[i], vcinfo[2*i], vcinfo[2*i+1]);
		} else {
			lpc_printf("- %03u\toffline\r\n", rsnodes[i]);
		}
	}
}

#ifndef GATEWAY
extern bool app_inactive;
#endif

extern FATFS Fatfs;
static char line[LINELEN];

#define _DEBUG_               0
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

static void rstimer(TimerHandle_t pxTimer)
{
	RemoteCommand_stop(rcid);
	//vTaskDelay(3000);
	if (rcid == RC_CMD_RS_GETDATA) 
	{
		lpc_printf("- data from node %03u is missing (timeout)\r\n", rsnodes[rsidx]);
	} else {
		lpc_printf("- node %03u: command timed out\r\n", rsnodes[rsidx]);
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

#ifndef GATEWAY
static void rstimer_sn(TimerHandle_t pxTimer)
{
	lpc_printf("ERROR: rstimer_sn - ReliableComm is hung; resetting\r\n");
	die();
}

#else

void RemoteSensing_menu(void)
{
	if (Util_Wakeup(rsnodes, &rsncnt) != SUCCESS) {
		lpc_printf("ERROR: failed to wake up nodes; resetting...\r\n");
	}	else if (RemoteSensing_start() != SUCCESS) {
		lpc_printf("ERROR: RemoteSensing start failed; resetting...\r\n");
	}
}

int doreadcfg(const char *fn)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	char *nline, *str;
	uint32_t time, rate, channels, ncnt, node;
	int i;

	// go to config dir
	ret = f_chdir("/Xnode");
	if (ret) {
		#if _DEBUG_
			PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// read config file
	ret = f_open(&file, fn, FA_READ);
	if (ret) {
		#if _DEBUG_
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// sampling time
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SAMPLINGTIME = %u", &time) != 1 || time > 0xFFFF) {
		PRINTF("Error reading sampling time\r\n");
		f_close(&file);
		return FAIL;
	}
	rstime = (uint16_t)time;
	PRINTF("SAMPLINGTIME = %u\r\n", rstime);

	// sampling rate
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SAMPLINGRATE = %u", &rate) != 1 || rate >= 0xFFFF) {
		PRINTF("Error reading sampling rate\r\n");
		f_close(&file);
		return FAIL;
	}
	if (rate != 500 && rate != 200 && rate != 100 && rate != 50 && rate != 20 && rate != 10) {
		lpc_printf("ERROR: invalid sampling rate specified; defaulting to 100Hz.\r\n");
		rate = 100;
	}
	rsrate = (uint16_t)rate;
	PRINTF("SAMPLINGRATE = %u\r\n", rsrate);

	// number of channels
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "NUMCHANNELS = %u", &channels) != 1 || channels < 1 || channels > 8) {
		PRINTF("Error reading number of channels\r\n");
		f_close(&file);
		return FAIL;
	}
	rschannels = (uint8_t)channels;
	PRINTF("NUMCHANNELS = %u\r\n", rschannels);

	// number of nodes
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "NUMNODES = %u", &ncnt) != 1 || ncnt > MAX_NODES) {
		PRINTF("Error reading number of nodes\r\n");
		f_close(&file);
		return FAIL;
	}
	rsncnt = (uint8_t)ncnt;
	PRINTF("NUMNODES = %u\r\n", rsncnt);

	// node ids
	nline = sdmalloc(512);
	configASSERT(nline);
	if (f_gets(nline, 512, &file) == NULL) {
		PRINTF("Error reading node ids\r\n");
		f_close(&file);
		return FAIL;
	}
	PRINTF("%s\r\n", nline);

	// parse nodeids
	str = strchr(nline, '=');
	if (str == NULL) {
		f_close(&file);
		return FAIL;
	}
	str++;
	str = strtok(str, " ,\r\n");
	for (i = 0; i < rsncnt && str != NULL; ++i, str = strtok(NULL, " ,\r\n")) {
		if (sscanf(str, "%u", &node) != 1 || node < 1 || node >= 0xFFFF) {
			break;
		}
		rsnodes[i] = (uint16_t)node;
	}
	sdfree(nline);
	f_close(&file);
	if (i != rsncnt) {
		return FAIL;
	}
	return SUCCESS;
}
#ifndef FRA	// FRA version has these number saved in xnode.cfg, so no need to do this step
static int readcfg(void)
{
	return doreadcfg("RemoteSensing.cfg");
}
#endif
#endif

#ifndef FRA	// For FRA version, idx is saved in NAND flash too
static int readidx(void)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	uint32_t last;

	data_index = 0;

	// go to config dir
	ret = f_chdir("/Xnode");
	if (ret) {
		#if _DEBUG_
			PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// read config file
	ret = f_open(&file, "Data.cfg", FA_READ);
	if (ret) {
		#if _DEBUG_
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// last index
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "LAST = %u", &last) != 1) {
		PRINTF("Error reading sampling time\r\n");
		f_close(&file);
		return FAIL;
	}
	data_index = last;
	PRINTF("LAST DATA = %u\r\n", last);

	data_index++;

	// write config file
	ret = f_open(&file, "Data.cfg", FA_WRITE | FA_CREATE_ALWAYS);
	if (ret) {
		#if _DEBUG_
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// last index
	snprintf(line, LINELEN, "LAST = %u\n", data_index);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data config\r\n");
		f_close(&file);
		return FAIL;
	}

	f_close(&file);
	return SUCCESS;
}

#else
int readidx(void)
{
	FRESULT ret;              // Result code
  FIL file;                 // File object
	uint32_t last, i;
  uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	data_index = 0;

  // try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
  if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_DATACFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
  0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
    PRINTF("NAND Flash read success\r\n");
		last = flashbuf[0];		for (i = 1;i<sizeof(last);i++) {last = last | flashbuf[i] <<(8*i);}
		if (last>50000) {last = 0;}	// prevent things go crazy (\)
	} else {
    PRINTF("NAND Flash read failed\r\n");
  }
	
	// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
  
	// go to config dir	
	ret = f_chdir("/Xnode");
  if (ret) {
    PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
	} else {
		// read config file
		ret = f_open(&file, "Data.cfg", FA_READ);
		if (ret) {
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		} else {
			// last index
			if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "LAST = %u", &last) != 1) {
				PRINTF("Error reading sampling time\r\n");
				f_close(&file);
			}
		}
	}
	PRINTF("LAST = %u\r\n", last);
	data_index = last;
	last = last + 1;

	// write config file
	if (!ret) {
  ret = f_open(&file, "Data.cfg", FA_WRITE | FA_CREATE_ALWAYS);
  if (ret) {
    PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
	//	return FAIL; // keep going
	} else {
		// last index
	snprintf(line, LINELEN, "LAST = %u\n", last);
	f_close(&file);
	if (f_puts(line, &file) == 0) {
    PRINTF("Error writing data config\r\n");
		f_close(&file);
		//return FAIL; // keep going
	}
	}
	}

	// write to NAND flash
	memcpy(flashbuf, &last, sizeof (last));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_DATACFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_DATACFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}
	
	return SUCCESS;
}
#endif

#ifndef FRA		//	FRA version doesn't really use RemoteSensing_writedata (data is not de-coded before writing to gateway node)
int RemoteSensing_writedata(SensorData *sd, uint16_t nodeid, uint32_t index)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	uint32_t i,j;
	uint8_t year, month, date, hour, minute, second;
	// Get time
	gettime(&year, &month, &date, &hour, &minute, &second);

	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object
	f_mount(0, &Fatfs);
	if (index == 0) {
		readidx();
		index = data_index;
	}

#ifndef GATEWAY 	// then current time ~ time that data was collected. Can move up to make it closer to sensing time
	sd->month = month;
	sd->date = date;
	sd->year = year;
	sd->hour = hour;
	sd->minute = minute;
	sd->second = second;
#endif
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object
	f_mount(0, &Fatfs);

	// go to data dir
	ret = f_chdir("/Data");
	if (ret) {
		#if _DEBUG_
			PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}
	// write data file
	snprintf(line, LINELEN, "data_%03u_%02u%02u%02u_%02u%02u%02u_%04u.txt", nodeid,year,month,date,hour,minute,second,index);	// nodeid_yyddmm_hhmmss_index
	ret = f_open(&file, line, FA_WRITE | FA_CREATE_ALWAYS);
	if (ret) {
		#if _DEBUG_
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}
	PRINTF("File: %s\n\r",line);

	// write time
	snprintf(line, LINELEN, "mm/dd/yr: %02d/%02d/20%02d; hr:mm:sc: %02d:%02d:%02d\r\n", sd->month, sd->date, sd->year, sd->hour, sd->minute, sd->second);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// write voltage and charging current
	snprintf(line, LINELEN, "Voltage: %.2f\tCurrent: %.2f\r\n",sd->voltage, sd->current);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// write time stamp
	snprintf(line, LINELEN, "Time stamp for 1st raw data sample: %lld\r\n",sd->timestamp);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// write titles
	len = snprintf(line, LINELEN, "Time (ms)");
	for (j = 0; j < rschannels; ++j) {
		len += snprintf(line + len, LINELEN - len, "\tCh.%u (%s)", j + 1, SensorUnit[channtype[j]]);
	}
	snprintf(line + len, LINELEN - len, "\n");
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// write data
	for (i = 0; i < sd->channels[0].sampSize; i++) {
		len = snprintf(line, LINELEN, "%6u", (1000 / rsrate) * i);
		for (j = 0; j < rschannels; ++j) {
			if (j < 3) {
				len += snprintf(line + len, LINELEN - len, "\t\t%.3f", sd->channels[j].sampData[i]); // using 1 \t doesn't look good
			} else {
				len += snprintf(line + len, LINELEN - len, "\t\t%.6f", sd->channels[j].sampData[i]);
			}
		}
		snprintf(line + len, LINELEN - len, "\n");
		if (f_puts(line, &file) == 0) {
			PRINTF("Error writing data\r\n");
			f_close(&file);
			return FAIL;
		}
	}

	f_close(&file);
	return SUCCESS;
}


#else
int RemoteSensing_writedata(SensorData *sd, uint16_t nodeid, uint32_t index)
{
#ifdef GATEWAY
	FRESULT ret;              // Result code
  FIL file;                 // File object
  uint32_t i;
	uint32_t count;
#endif	
  uint8_t *flashbuf;
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
#ifdef SDCARD	
	// mount sdcard
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	
	if (index == 0) {
		readidx();
		index = data_index;
	}
	
  // go to data dir	
	ret = f_chdir("/Data");
  if (ret) {
    PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		return FAIL;
	}

	// write data file
	snprintf(line, LINELEN, "data_%04u_%06u.txt", nodeid, index);
  ret = f_open(&file, line, FA_WRITE | FA_CREATE_ALWAYS);
  if (ret) {
    PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		return FAIL;
	}
	
	// write time
	snprintf(line, LINELEN, "mm/dd/yr: %2d/%2d/20%2d; hr:mm:sc: %02d:%02d:%02d\r\n", sd->month, sd->date, sd->year, sd->hour, sd->minute, sd->second);
	if (f_puts(line, &file) == 0) {
		f_close(&file);
		return FAIL;
	}
	
	// write voltage and charging current
	snprintf(line, LINELEN, "Voltage: %.2f\tCurrent: %.2f\r\n",sd->voltage, sd->current);
	if (f_puts(line, &file) == 0) {
		f_close(&file);
		return FAIL;
	}
	
  // write titles
	snprintf(line, LINELEN, "Time (ms)\tAccelX (mg)\tAccelY (mg)\tAccelZ (mg)\n");
	if (f_puts(line, &file) == 0) {
		f_close(&file);
		return FAIL;
	}
	
	for (i = 0; i < sd->length; i++) {
		snprintf(line, LINELEN, "%6u\t%.6f\t%.6f\t%.6f\n",1000/sd->channels[0].samplingRate*i,sd->channels[0].sampData[i],sd->channels[1].sampData[i],sd->channels[2].sampData[i]);
		if (f_puts(line, &file) == 0) {
			f_close(&file);
			return FAIL;
		}
	}
  f_close(&file);
#else

#endif
	lpc_printf("Done writing...\r\n");
#ifdef GATEWAY
	
 // try NAND Flash config first
	PRINTF("Reading NAND Flash...\r\n");
  if (NandFlash_PageRead(flashbuf, NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
  0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
    PRINTF("NAND Flash read success\r\n");
		count = flashbuf[0];		for (i = 1;i<sizeof(count);i++) {count = count | flashbuf[i] <<(8*i);}
		if (count>50000) {count = 0;}	// prevent things go crazy (\)
	} else {
    PRINTF("NAND Flash read failed\r\n");
  }
	
	// go to data dir	
	ret = f_chdir("/Xnode");
  if (ret) {
    PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
	//	return FAIL;
	} else {
	// data to send count
	ret = f_open(&file, "DataToSendCount.cfg", FA_READ);
  if (ret) {
    PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
	//	return FAIL;
	} else {	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "COUNT = %u", &count) != 1) {
    PRINTF("Error reading sampling time\r\n");
		f_close(&file);
	//	return FAIL;
	}
		f_close(&file);
	}
	
	ret = f_open(&file, "DataToSendCount.cfg", FA_WRITE | FA_CREATE_ALWAYS);
  if (ret) {
    PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
	//	return FAIL;
	} else {		
	snprintf(line, LINELEN, "COUNT = %u", count+1);
	if (f_puts(line, &file) == 0) {
    PRINTF("Error writing data\r\n");
		f_close(&file);
	//	return FAIL;
	}
	
	lpc_printf("WROTE %s \n\r",line);
	f_close(&file);
	}
}
	// write to NAND flash
	count = count + 1;
	memcpy(flashbuf, &count, sizeof (count));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_GATEWAY_DATATOSENDCOUNTCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}
	
#endif
	
	return SUCCESS;
}
#endif

int PackAndSave(SensorData *sd)
{
	uint32_t i;
	uint32_t sdidx, len;
	uint8_t *data;
	uint8_t year, month, date, hour, minute, second;
	configASSERT(sd);

	gettime(&year, &month, &date, &hour, &minute, &second);

#if defined(GW_SENSE) || !defined(GATEWAY) 	// this PackAndSave function is not used in GATEWAY anyways, but just to be sure. Fixed: It is used now with GW_SENSE
  CheckBatteryVoltage(); // TUFIXDONE: First reading of voltage and current, without this, these 2 values will be zero
	sd->voltage = voltage; // put this here as triggersensing doesn't have time to check these information like remotesensing
	sd->current = current; // put this here as triggersensing doesn't have time to check these information like remotesensing
	sd->month = month;
	sd->date = date;
	sd->year = year;
	sd->hour = hour;
	sd->minute = minute;
	sd->second = second;
#endif
	// assume sensing 3 channels with same data rate
	for (i = 0, sdidx = 0; i < rschannels; i++) {
		sdidx += dslen * sizeof (float);
	}
#ifdef FRA // add space for  adxl data
	for (i = 0; i < 3; i++) { //adxlData
		sdidx += SAMPLE_SET * sizeof (int16_t);
	}
#endif	
	len = 2 * sizeof (uint32_t) + sizeof(uint64_t) + sizeof (SensorData) + sdidx;
	// assure correct memory alignment
	len += 3;
	len >>= 2;
	len <<= 2;
	data = (uint8_t *)sdcalloc(1,len);
	configASSERT(data); memset(data, 0, len);

	*(uint32_t *)data = len;
	memcpy(data + sizeof (uint32_t), sd, sizeof (SensorData)); // what is this uint32_t used for?
	sdidx = sizeof (uint32_t) + sizeof (SensorData);

	for (i = 0; i < rschannels; i++) {
		len = dslen * sizeof(float);
		memcpy(&data[sdidx], sd->channels[i].sampData, len);
		sdidx += len;
	}
#ifdef FRA // add disp and adxl data
	for (i = 0; i < 3; i++) {
		len = SAMPLE_SET * sizeof(int16_t);
		memcpy(&data[sdidx], sd->channels[i].adxlData, len);
		sdidx += len;
	}
#endif	
	
#ifndef FRA
	return(WriteToNAND(data, &sdidx));
#else // write data to both NAND and SD for redundancy
	// write to NAND flash
	WriteToNAND(data, &sdidx);
	// write to sd card
	WriteToSD(data, &sdidx);
	return SUCCESS;
#endif	
}

int WriteToNAND(uint8_t *data, uint32_t* sdidx) //TUFIX: More Elegant way to delete full NAND
{
	int f;
	CHAR filename[121], path[121];
	YCHAR *restOfPath = (YCHAR*)sdmalloc(YAFFS_MAX_NAME_LENGTH + 1); // need this malloc here or str[i] = *restOfPath; causes trouble
	uint8_t year, month, date, hour, minute, second;
	configASSERT(restOfPath);
	yaffs_start_up(); // init later so full NAND doesn't affect wake up time
	yaffs_mount("/nand");
	if (isformat) // TUFIX: when full needs to be formated. it doesn't relocate itself. Should do in a way that no data is lost TUFIX - easiest way: dumping all files to sd card
	{
		if (read_nand2sdattemp() == 0) 
		{
			NAND2SD(); // always dump all files (900 blocks) to SD, regardless, before formatting
		}
		if (yaffs_cleanOldFiles() == 0)
		{
			NAND2SD();
			yaffs_format("/nand",1,1,1);
		}
	} 
	// Get time
	gettime(&year, &month, &date, &hour, &minute, &second);
	snprintf(filename, LINELEN, "data_%03u_%02u%02u%02u_%02u%02u%02u.txt", LOCAL_ADDR,year,month,date,hour,minute,second);	// nodeid_yyddmm_hhmmss_index
	snprintf(path, 120, "/nand/%s",filename);
	lpc_printf("path: %s\n\r",path);
	f = yaffs_open(path, O_CREAT | O_TRUNC | O_RDWR, (S_IREAD | S_IWRITE));
	if(f == -1)
	{
		PRINTF("Open/Create file error\n\r");
		return FAIL;
	}
	if(yaffs_write(f, data, (unsigned int) *sdidx) == -1)
	{
		PRINTF("Write error\n\r");
		return FAIL;
	}

	if(yaffs_flush(f)== -1)
	{
		PRINTF("Flush error\n\r");
		return FAIL;
	}
	if(yaffs_close(f)== -1)
	{
		PRINTF("Close error\n\r");
		return FAIL;
	}
	lpc_printf("- done writing\n\r");
	return SUCCESS;
}

#ifdef FRA	// FRA code also writes binary data to sd card of sensor node (not just NAND flash of sensor node)
int WriteToSD(uint8_t *data, uint32_t* sdidx)
{
	uint32_t dlen = 0;
	FRESULT ret;              // Result code
	FIL file;                 // File object 	
	UINT len_fwrite;
	uint8_t year, month, date, hour, minute, second;

	readidx();
  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	PRINTF("- writing binary data to sdcard...");
	ret = f_chdir("/Data");
	if (ret) {
    PRINTF("ERROR: Fail to open directory\r\n");
		return FAIL;
	}

	// write data file
	gettime(&year, &month, &date, &hour, &minute, &second);
	snprintf(line_write, LINELEN, "data_%03u_%02u%02u%02u_%02u%02u%02u.txt", LOCAL_ADDR,year,month,date,hour,minute,second);	// nodeid_yyddmm_hhmmss_index
  ret = f_open(&file, line_write, FA_WRITE | FA_CREATE_ALWAYS);
  if (ret) {
    PRINTF("ERROR: Create a new file error\n");
		return FAIL; //TUFIXDONE: Original code doesn't have this. Check if this is ok
	}

	len_fwrite = 0;
	dlen = *sdidx;
	do
	{
		dlen = dlen - len_fwrite;
		f_write(&file, data + len_fwrite, dlen, &len_fwrite);
	} while (len_fwrite > 0);
	f_close(&file);
	
	return SUCCESS;
}
#endif

static void packSensorData(SensorData *sd, uint32_t max, uint8_t **data, uint32_t *size)
{
	uint8_t i;
	uint32_t sdidx, len;
	configASSERT(sd && data && size);

	for (i = 0, sdidx = 0; i < rschannels; i++) {
		sdidx += sd->channels[i].sampSize * sizeof (float);
	}
	len = 2 * sizeof (uint32_t) + sizeof(uint64_t) + sizeof (SensorData) + sdidx;
	// assure correct memory alignment
	len += 3;
	len >>= 2;
	len <<= 2;
	*data = (uint8_t *)sdcalloc(len, 1);
	configASSERT(*data);
	*size = len;

	*(uint32_t *)*data = len;
	memcpy(*data + sizeof (uint32_t), sd, sizeof (SensorData));
	sdidx = sizeof (uint32_t) + sizeof (SensorData);

	for (i = 0; i < rschannels; i++) {
		if (sd->channels[i].sampSize == 0) {
			continue;
		}
		len = sd->channels[i].sampSize * sizeof (float);
		memcpy(*data + sdidx, sd->channels[i].sampData, len);
		sdidx += len;
	}

	*(uint32_t *)(*data + *size - sizeof (uint32_t) - sizeof (uint64_t)) = max;
}

static uint32_t unpackSensorData(uint8_t *data, SensorData **sd, uint32_t *max)
{
	uint8_t i;
	uint32_t sdidx, len;

	configASSERT(data && sd && max);

	len = *(uint32_t *)data;
	*max = *(uint32_t *)(data + len - sizeof (uint32_t) - sizeof(uint64_t));

	*sd = (SensorData *)sdcalloc(len - 2 * sizeof (uint32_t) - sizeof(uint64_t), 1);
	configASSERT(*sd);
	memcpy(*sd, data + sizeof(uint32_t), len - 2 * sizeof (uint32_t) - sizeof(uint64_t));
	sdidx = sizeof (SensorData);
	for (i = 0; i < rschannels; i++)
	{
		if ((*sd)->channels[i].sampSize == 0) {
			continue;
		}
		(*sd)->channels[i].sampData = (float *)((uint8_t *)*sd + sdidx);
		sdidx += (*sd)->channels[i].sampSize * sizeof (float);
	}
	return (*sd)->channels[0].sampSize;
}

void sensingFunc(SensorData *data)
{
	/*
	lpc_printf("- writing data to SD card...\r\n");
	if (RemoteSensing_writedata(data, LOCAL_ADDR, 0) != SUCCESS) {
		lpc_printf("ERROR: failed to write data to SD card\r\n");
	}
	*/
	lpc_printf("- writing data to NAND flash...\r\n");
	if (PackAndSave(data) != SUCCESS) {
		lpc_printf("ERROR: failed to write data to NAND flash\r\n");
	}
	lpc_printf("- finished writing data.\r\n");
	LED_Off();
	if (radio_init() != SUCCESS) {
		lpc_printf("ERROR: failed to init radio\r\n");
		die();
	}
	if(TrigSenIdx!=0) {
		adxl362_setup();
	}
	radio_set_short_addr(LOCAL_ADDR);
	dataready = true;
}

static int setparamsfunc(void* arg, uint32_t len)
{
	uint8_t i;
	sumessage *sm = (sumessage *)arg;
	int res = SUCCESS;
#ifndef GATEWAY
	app_inactive = false;
#endif

	if (len != sizeof (sumessage)) {
		res = FAIL;
	}
	if (res == FAIL) {
		RemoteCommand_done(RC_CMD_RS_SETPARAMS, FAIL, NULL, 0);
		xTimerStop(tim_sn, portMAX_DELAY);
		xTimerChangePeriod(tim_sn, pdMS_TO_TICKS(30000), portMAX_DELAY);
		portYIELD();
		return FAIL;
	}

	RemoteCommand_done(RC_CMD_RS_SETPARAMS, SUCCESS, NULL, 0);
	portYIELD();
	radio_sleep();
	rschannels = (int) sm->channelMask;
	// init of sensor module
	ads131_init();
	ads131_on();
	sd = setupAcquisition(sm->dataRate, sm->sampleTime, sm->time64);
	if (!sd) {
		die();
		return FAIL;
	}

	for (i = 0; i < rschannels; i++) {
		if (setupChannel(sd, i, sm->samplingRate, sm->ctype) == ERROR) {
			die();
			return FAIL;
		}
	}

	xTimerStop(tim_sn, portMAX_DELAY);
	xTimerChangePeriod(tim_sn, 60*60000UL, portMAX_DELAY);
	dataready = false;
	startAcquisition(sd, sensingFunc);
	return SUCCESS;
}

static void setparamssent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		//vTaskDelay(3000);
		lpc_printf("ERROR: all nodes unresponsive\r\n");
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void setparamsresp(int success)
{
	if (success != SUCCESS) {
		lpc_printf("ERROR: failed to set parameters\r\n");
		die();
	}
}

static void setparamsexec(int success, void *retval, uint32_t len)
{
	if (success != SUCCESS) {
		//vTaskDelay(3000);
		lpc_printf("ERROR: all nodes unresponsive\r\n");
	} else {
		spsuccess = true;
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

static int getdatafunc(void* arg, uint32_t len)
{
	uint8_t i;
	uint8_t *dptr = NULL;
	uint32_t dlen = 0;
	int res = SUCCESS;

	xTimerStop(tim_sn, portMAX_DELAY);
	xTimerChangePeriod(tim_sn, 5*60000, portMAX_DELAY);

	if (!dataready || !sd) {
		res = FAIL;
	} else {
		packSensorData(sd, rsrate * rstime, &dptr, &dlen);
		if (dptr == NULL || dlen == 0) {
			res = FAIL;
			dptr = NULL;
			dlen = 0;
		}
	}

	for (i = 0; i < rschannels; ++i) {
		sdfree(sd->channels[i].sampData);
	}
	sdfree(sd);
	sd = NULL;

	lpc_printf("- sending back data (%u bytes)...\r\n", dlen);
	RemoteCommand_done(RC_CMD_RS_GETDATA, res, dptr, dlen);
	portYIELD();

	return res;
}

static void getdatasent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		//vTaskDelay(5000);
		lpc_printf("- data from node %03u is missing (timeout)\r\n", rsnodes[rsidx]);
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void getdataresp(int success)
{
	LED_Off();
	vTaskDelay(500);
	NodeReset();
}

static void processdatatask(void *pvParameters)
{
#ifdef PRINTDATA
	uint32_t i, j;
#endif
	//uint32_t max = (uint32_t)pvParameters;
	(void)pvParameters;
	vcinfo[2*rsidx] = sd->voltage;
	vcinfo[2*rsidx+1] = (sd->current < 120.f) ? 0.f : sd->current;
	//lpc_printf("- node %03u: battery voltage is %.2fV, charging current is %.2fmA\n\r", rsnodes[rsidx], vcinfo[2*rsidx], vcinfo[2*rsidx+1]);
	lpc_printf("- writing data to SD card...\r\n");
	if (RemoteSensing_writedata(sd, rsnodes[rsidx], data_index) != SUCCESS) {
		lpc_printf("ERROR: failed to store data\r\n");
	}
#ifdef PRINTDATA
	lpc_printf("\r\ndata_%04u_%06u.txt\r\n", rsnodes[rsidx], data_index);
	lpc_printf("Time (ms)       ");
	for (j = 0; j < rschannels; ++j) {
		if (j == 0){
			lpc_printf("Ch.%u (%s)", j + 1, SensorUnit[channtype[j]]);
		}
		else {
			lpc_printf("\t\tCh.%u (%s)", j + 1, SensorUnit[channtype[j]]);
		}
	}
	lpc_printf("\r\n");

	for (i = 0; i < sd->channels[0].sampSize; i++) {
		lpc_printf("%6u", (1000 / rsrate) * i);
		for (j = 0; j < rschannels; ++j) {
			if (j < 3) {
				lpc_printf("\t\t%.3f", sd->channels[j].sampData[i]);
			} else {
				lpc_printf("\t\t%.6f", sd->channels[j].sampData[i]);
			}
		}

		lpc_printf("\r\n");
	}
#endif
	lpc_printf("\r\n");

	sdfree(sd);
	sd = NULL;

	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
		//portYIELD();
	}
	vTaskDelete(NULL);
}

static void processdata(uint8_t *data, uint32_t len)
{
	uint32_t max;
	len = unpackSensorData(data, &sd, &max);
	if (!sd || len == 0) {
		lpc_printf("ERROR: failed to unpack data\r\n");
		return;
	}
	if (xTaskCreate(processdatatask, "ProcData", 5 * configMINIMAL_STACK_SIZE, (uint32_t *)max,
	(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *)NULL) != pdPASS) {
		lpc_printf("ERROR: failed to process data\r\n");
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
			//portYIELD();
		}
	}
}

static void getdataexec(int success, void *retval, uint32_t len)
{
	xTimerStop(tim, portMAX_DELAY);
	if (success != SUCCESS) {
		//vTaskDelay(3000);
		lpc_printf("- data from node %03u is missing (no response)\r\n", rsnodes[rsidx]);
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
			//portYIELD();
		}
	} else {
		rsok++;
		lpc_printf("- data received from node %03u\r\n", rsnodes[rsidx]);
		processdata((uint8_t *)retval, len);
	}
}

int RemoteSensing_init(void)
{
	if (initialized) {
		return SUCCESS;
	}
	if (RemoteCommand_init() != SUCCESS
	|| RemoteCommand_register(RC_CMD_RS_SETPARAMS, false, setparamsfunc, setparamssent, setparamsresp, setparamsexec) != SUCCESS
	|| RemoteCommand_register(RC_CMD_RS_GETDATA, true, getdatafunc, getdatasent, getdataresp, getdataexec) != SUCCESS) {
		return FAIL;
	}
	Timesync_Init();
#ifndef FRA	// FRA version has these number saved in xnode.cfg, so no need to do this step
	#ifdef GATEWAY
		readcfg();
	#else
		// defaults
		rschannels = MAX_ACCEL_CHANNELS;
		rstime = 180;
		rsrate = 100;
	#endif
#endif

#ifndef GATEWAY
	tim_sn = xTimerCreate("RSTimer", pdMS_TO_TICKS(6*60000), pdFALSE, NULL, rstimer_sn);
	configASSERT(tim_sn);
	xTimerStart(tim_sn, portMAX_DELAY);
#endif

	initialized = true;
	return SUCCESS;
}

int RemoteSensing_start(void)
{
	float ttemp;
	if (!initialized) {
		return FAIL;
	}

	lpc_printf("\r\n- starting time sync round 1; wait 5 seconds...\r\n");
	TimeSync_Start();
	//vTaskDelay(5000);

	tim = xTimerCreate("RSTimer", pdMS_TO_TICKS(30000), pdFALSE, NULL, rstimer);
	configASSERT(tim);
	memset(&sumsg, 0, sizeof (sumessage));
	sumsg.sampleTime = rstime;
	sumsg.samplingRate = rsrate;
	sumsg.dataRate = RAW_DATA_RATE;
	sumsg.time64 = GlobalTime_get64()+2*120*1e6; // offset by 2s.

	//lpc_printf("time to start sensing is %lld\r\n", sumsg.time64);

	sumsg.channelMask = (int) rschannels;
	sumsg.ctype = XNODEACCEL;
	vcinfo = (float *)sdcalloc(2 * MAX_NODES, sizeof (float));
	configASSERT(vcinfo);

	lpc_printf("- sending parameters to %u sensor nodes; wait %u seconds...\r\n", rsncnt, 3);
	spsuccess = false;
	rcid = RC_CMD_RS_SETPARAMS;
	xTaskToNotify = xTaskGetCurrentTaskHandle();

	xTimerStart(tim, portMAX_DELAY);
	if (RemoteCommand_execute(RC_CMD_RS_SETPARAMS, rsnodes, rsncnt, &sumsg, sizeof (sumessage)) != SUCCESS) {
		lpc_printf("ERROR: all nodes unresponsive\r\n");
	} else {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}

	xTimerStop(tim, portMAX_DELAY);
	rcid = 0xff;
	if (!spsuccess) {
		xTimerDelete(tim, portMAX_DELAY);
		xTaskToNotify = NULL;
		return FAIL;
	}

	lpc_printf("- sensing parameters sent to all nodes.\r\n");
	vTaskDelay(1000);
	lpc_printf("- sensing started; wait %u seconds...\r\n", rstime+5);
	vTaskDelay(rstime * 1000+2000); // Yuguang Comments: 2s delay is for the

	lpc_printf("- starting time sync round 2; wait 5 seconds...\r\n");
	TimeSync_Start();
	vTaskDelay(5000);

	//ttemp = 60.f / rsrate + (0.12f*rstime + 0.8f*rschannels) * (0.01f*rsrate);
	ttemp = 60.f / rsrate + (0.12f*rstime + 0.8f*rschannels) * (0.01f*rsrate) / 2;
	lpc_printf("- sensing finished, processing and storing data; wait %u seconds...\r\n", (int)ttemp);
	vTaskDelay((int)ttemp * 1000);
	LED_Off();

	//vTaskDelay(60000); // for additional raw data processing

	lpc_printf("\r\n- requesting data from %u sensor nodes...\r\n", rsncnt);
	rsidx = rsok = 0;
	rcid = RC_CMD_RS_GETDATA;
	for (rsidx = 0, rsok = 0; rsidx < rsncnt; ++rsidx) {
		LED_RGB(0,0,1);
		lpc_printf("\r\n- requesting data from node %03u...\r\n", rsnodes[rsidx]);
		xTimerChangePeriod(tim, 60000, portMAX_DELAY);
		if (RemoteCommand_execute(RC_CMD_RS_GETDATA, rsnodes + rsidx, 1, NULL, 0) != SUCCESS) {
			lpc_printf("- data from node %03u is missing (command failed)\r\n", rsnodes[rsidx]);
			//vTaskDelay(3000);
		} else {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
		xTimerStop(tim, portMAX_DELAY);
		LED_Off();
		//vTaskDelay(2000);
	}
	xTimerDelete(tim, portMAX_DELAY);
	xTaskToNotify = NULL;
	rcid = 0xff;

	Util_ResetNodes(rsnodes, rsncnt);

	if (rsok == 0) {
		lpc_printf("ERROR: all nodes unresponsive; RemoteSensing failed!\r\n");
	} else {
		print_vcinfo();
		lpc_printf("- data received from %u nodes, %u nodes missing\r\n", rsok, rsncnt - rsok);
	}

	lpc_printf("- resetting...\r\n");
	vTaskDelay(500);
	return (rsok == 0);
}

int write_acc(float *output[], uint32_t size)    ///////// check the index  //TUFIXDONE: FIX for the saving 1kHz data case (binary to SD is still fast). Fix: Use packSensorDataRAW
{
	// Write sensing data (before decimation) to SD card (of leaf node)
	FRESULT ret;              // Result code
	FIL file;                 // File object
	uint32_t i, index, j;

	lpc_printf("Writing raw data...");

	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object
	f_mount(0, &Fatfs);

	readidx();
	index = data_index;

	ret = f_chdir("/Data");
	if (ret) {
		lpc_printf("Change directory error\n");
		return FAIL;
	}

	len = snprintf(line_write, 80, "original_Xnode_data_%06u.txt", index);
	ret = f_open(&file, line_write, FA_WRITE | FA_CREATE_ALWAYS);
	if (ret) {
		PRINTF("Error creating new file\r\n");
		return FAIL;
	}

	// write data
	len = snprintf(line, LINELEN, "Time (ms)");
	for (j = 0; j < rschannels; ++j) {
		len += snprintf(line + len, LINELEN - len, "\tCh.%u (%s)", j + 1, SensorUnit[channtype[j]]);
	}
	snprintf(line + len, LINELEN - len, "\n");

	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	for (i = 0; i < size; i++) {
		len = snprintf(line, LINELEN, "%6u", i);
		for (j = 0; j < rschannels; ++j) {
			len += snprintf(line + len, LINELEN - len, "\t%.6f", output[j][i]);
		}
		snprintf(line + len, LINELEN - len, "\n");
		if (f_puts(line, &file) == 0) {
			PRINTF("Error writing data\r\n");
			f_close(&file);
			return FAIL;
		}
	}

	f_close(&file);
	lpc_printf("Done\r\n");

	return SUCCESS;
}

#ifdef FRA	// function to write raw data to SD Card of sensor node, for saturation cases. (TUFIX: Check data)
void packSensorDataRAW(float **sd, uint32_t numchan, uint32_t numsam)
{
	uint8_t i;
	uint32_t dlen = 0;
	FRESULT ret;              // Result code
	FIL file;                 // File object 	
	UINT len_fwrite;
	uint8_t *data;
	uint8_t year, month, date, hour, minute, second;

	readidx();
	data = (uint8_t *)sdcalloc(numsam*numchan*4, 1);
	configASSERT(data);
	for (i = 0; i < numchan; i++) {
		memcpy(data + i*numsam*4, sd[i], numsam*4);
	}

  memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	lpc_printf("Writing raw binary data to sdcard...");
	ret = f_chdir("/Data");
	if (ret) {
    lpc_printf("FAIL\r\n");
	} else
	{
		// write data file      
		gettime(&year, &month, &date, &hour, &minute, &second);
		snprintf(line_write, LINELEN, "RawData_%03u_%02u%02u%02u_%02u%02u%02u.txt", LOCAL_ADDR,year,month,date,hour,minute,second);	// nodeid_yyddmm_hhmmss_index
		ret = f_open(&file, line_write, FA_WRITE | FA_CREATE_ALWAYS);
		if (ret) {
			lpc_printf("Create a new file error\n");
		}

		len_fwrite = 0;
		dlen = numsam*numchan*4;
		do
		{
			dlen = dlen - len_fwrite;
			f_write(&file, data + len_fwrite, dlen, &len_fwrite);
		} while (len_fwrite > 0);
		f_close(&file);
	}
}
#endif

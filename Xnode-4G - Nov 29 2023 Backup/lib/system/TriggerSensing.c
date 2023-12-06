/*************************************************
*@File        TriggerSensing.c
*@Brief       Functions for wake-up sensor
*@Version     1.0
*@Date        08/27/17
*@Author      Yuguang Fu
**************************************************/

#include <xnode.h>
#include <remotesensing.h>
#include <sensing.h>
#include <string.h>
#include <ads131.h>
#include <led.h>
#include <lpc43xx_evrt.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_pwr.h>
#include <Radio.h>
#include <ff.h>
#include "adxl362.h"
#include "triggersensing.h"
#include <stdio.h>
#include "rtc.h"
#include <RemoteCommand.h>
#include "Utils.h"
#include <nandflash_k9f1g08u0a.h>

#ifdef FRA  // Add sensing upper limit, parameterized
	extern uint32_t sulimit;
#endif 
/*---- TriggerSensing parameters by default ----*/
uint16_t datarate = 100;
uint8_t FIFO_mode = 3;  // trigger mode
uint16_t FIFO_num = SAMPLE_SET*3; // the range is from 0 to 511, TRY 170 SAMPLES for 3 Channels

/*---- TriggerSensing parameters read from SDcard ----*/
int adxl_rate;
uint16_t adxl_range;
uint16_t thres_act; // set activity threshold detection as 200mg (if range is 2g, LSB/g is 1000)
uint16_t time_act; // set time for activity detection as 50ms (if ODR is 100, time is 5/100s = 50ms)
uint16_t thres_iact; // set activity threshold detection as 200mg (if range is 2g, LSB/g is 1000)
uint16_t time_iact; // set time for activity detection as 50ms (if ODR is 100, time is 5/100s = 50ms)
uint16_t FIFO_numb;
uint8_t tschannels;
uint16_t tstime, tsrate;

/*---- TriggerSensing global variables ----*/
static bool initialized = false;
static TaskHandle_t xTaskToNotify = NULL;
static uint8_t rcid = 0xff;
#define LINELEN               150
static char line[LINELEN];

/*----  Shared functions/variables from other files ----*/
extern uint8_t TrigSenIdx;
extern FATFS Fatfs;
void sensingFunc(SensorData *data);
int SDCard_Init(void);
int SDCard_ReWrite(int trignum);

/*---- TriggerSensing parameters specified by users via RemoteCommand ----*/
typedef struct {
	uint32_t sampleTime;
	uint16_t samplingRate;
	uint8_t range;
	uint16_t thresholdact;
	uint16_t timeact;
	uint16_t thresholdinact;
	uint16_t timeinact;
	uint16_t FIFOnumb;
	uint8_t channelMask;
	uint8_t trigsen;
} __attribute__((packed)) tsmessage;
static tsmessage tsmsg;

/*---- TriggerSensing configuration parameters read/write in Nandflash ----*/
typedef struct {
	uint16_t adxlrange;
	uint16_t adxlthresholdact;
	uint16_t adxltimeact;
	uint16_t adxlthresholdinact;
	uint16_t adxltimeinact;
	uint16_t adxlfifonum;
	uint16_t adxlsamplingtime;
	uint16_t adxlsamplingrate;
	uint8_t adxlnumchannels;
} adxlconfig_t;

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
case FR_OK:
return "Succeeded";
case FR_DISK_ERR:
return "A hard error occured in the low level disk I/O layer";
case FR_INT_ERR:
return "Assertion failed";
case FR_NOT_READY:
return "The physical drive cannot work";
case FR_NO_FILE:
return "Could not find the file";
case FR_NO_PATH:
return "Could not find the path";
case FR_INVALID_NAME:
return "The path name format is invalid";
case FR_DENIED:
return "Acces denied due to prohibited access or directory full";
case FR_EXIST:
return "Acces denied due to prohibited access";
case FR_INVALID_OBJECT:
return "The file/directory object is invalid";
case FR_WRITE_PROTECTED:
return "The physical drive is write protected";
case FR_INVALID_DRIVE:
return "The logical drive number is invalid";
case FR_NOT_ENABLED:
return "The volume has no work area";
case FR_NO_FILESYSTEM:
return "There is no valid FAT volume on the physical drive";
case FR_MKFS_ABORTED:
return "The f_mkfs() aborted due to any parameter error";
case FR_TIMEOUT:
return "Could not get a grant to access the volume within defined period";
case FR_LOCKED:
return "The operation is rejected according to the file shareing policy";
case FR_NOT_ENOUGH_CORE:
return "LFN working buffer could not be allocated";
case FR_TOO_MANY_OPEN_FILES:
return "Number of open files > _FS_SHARE";
default:
return "unknown";
}
}
#endif

#ifdef GATEWAY
static const char trig_menu[] =
	"- Choose action:\r\n"
	"- '1'  enable trigger sensing\r\n"
	"- '2'  disable trigger sensing\r\n"
	"- '3'  Return\r\n"
	"- Xnode> ";

static const char params_menu[] = 
	"\r\n- Choose action:\r\n"
	"- '1'  Restore defaults\r\n"
	"- '2'  Change measurement range\r\n"
	"- '3'  Change threshold for activity detection\r\n"
	"- '4'  Change time for activity detection\r\n"
	"- '5'  Change threshold for inactivity detection\r\n"
	"- '6'  Change time for inactivity detection\r\n"
	"- '7'  Change sampling time\r\n"
	"- '8'  Change sampling rate\r\n"
	"- '9'  Change channel number\r\n"
	"- '0'  Return\r\n"
	"- Xnode> ";
#endif

static int readcfg_adxl(const char *fn)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	uint32_t range, acthreshold, actime, inacthreshold, inactime, FIFOnum, time, rate, channels;
	int res = FAIL;
	adxlconfig_t adxlcfg, *adxlcfgptr;
	uint8_t *flashbuf, *flashbuf1;
	char NameisTriggerSensing[] = "TriggerSensing.cfg";
	int block; // either TRIGGERSENSING or TRIGGERSENSINGDEFAULT position in NAND
// TUFIXDONE: This readcfg_adxl shouldn't be used in any FRA project, or it would cause rewriting of the thresholding values
#ifdef FRA
	return SUCCESS;
#endif
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	flashbuf1 = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf1);

	block = (strcmp(fn, NameisTriggerSensing)==0) ? NANDFLASH_BLOCK_TRIGGERSENSINGCFG: NANDFLASH_BLOCK_TRIGGERSENSINGDEFAULTCFG;

	// try NAND Flash config for TriggerSensing first
	PRINTF("Reading NAND Flash...\r\n");
	if (NandFlash_PageRead(flashbuf, block * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
		0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash read success\r\n");
		adxlcfgptr = (adxlconfig_t *)flashbuf;
		range = adxlcfgptr->adxlrange;
		acthreshold = adxlcfgptr->adxlthresholdact;
		actime = adxlcfgptr->adxltimeact;
		inacthreshold = adxlcfgptr->adxlthresholdinact;
		inactime= adxlcfgptr->adxltimeinact;
		FIFOnum = adxlcfgptr->adxlfifonum;
		time = adxlcfgptr->adxlsamplingtime;
		rate = adxlcfgptr->adxlsamplingrate;
		channels = adxlcfgptr->adxlnumchannels;
		res = SUCCESS;
	} else {
		PRINTF("NAND Flash read failed\r\n");
		res = FAIL;
	}
	
	adxl_range = (uint16_t)range; PRINTF("(NAND) RANGE = %u\r\n", adxl_range);

	switch (adxl_range)
	{
		case 2: adxl_rate=1; break;
		case 4: adxl_rate=2; break;
		case 8: adxl_rate=4; break;
		default: PRINTF("Error value of range\r\n"); break;
	}
	PRINTF("(NAND) adxl_rate is %u\r\n", adxl_rate);

	thres_act = (uint16_t)acthreshold/adxl_rate;
	PRINTF("(NAND) THRESHOLDACT = %u\r\n", thres_act);

	time_act = (uint16_t)actime;
	PRINTF("(NAND) TIMEACT = %u\r\n", time_act);

	thres_iact = (uint16_t)inacthreshold/adxl_rate;
	PRINTF("(NAND) THRESHOLDINACT = %u\r\n", thres_iact);

	time_iact = (uint16_t)inactime;
	PRINTF("(NAND) TIMEINACT = %u\r\n", time_iact);

	FIFO_numb = (uint16_t)FIFOnum;
	PRINTF("(NAND) FIFONUM = %u\r\n", FIFO_numb);

	tstime = (uint16_t)time;
	PRINTF("(NAND) SAMPLINGTIME = %u\r\n", tstime);

	tsrate = (uint16_t)rate;
	PRINTF("(NAND) SAMPLINGRATE = %u\r\n", tsrate);

	tschannels = (uint8_t)channels;
	PRINTF("NUMCHANNELS = %u\r\n", tschannels);
	
#ifdef GW_SENSE
	//adxl_rate = 1;
#endif
	// go to config dir	
	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);
	
	ret = f_chdir("/Xnode");
	
	if (ret) {
		PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		return res;
	}
	
	// read config file
	ret = f_open(&file, fn, FA_READ);
	if (ret) {
		PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		return res;
	}

	// measurement range
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "RANGE = %u", &range) != 1 || range > 8) {
		PRINTF("Error reading  measurement range\r\n");
		f_close(&file);
		return res;
	}
	adxl_range = (uint16_t)range; PRINTF("RANGE = %u\r\n", adxl_range);
	switch (adxl_range)
	{
		case 2: adxl_rate=1; break;
		case 4: adxl_rate=2; break;
		case 8: adxl_rate=4; break;
		default: PRINTF("Error value of range\r\n"); break;
	}
	PRINTF("adxl_rate is %u\r\n", adxl_rate);
	
#ifdef GW_SENSE
	//adxl_rate = 1;
#endif

	// threshold for activity detection
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "THRESHOLDACT = %u", &acthreshold) != 1 || acthreshold >= 0xFFFF) {
		PRINTF("Error reading threshold for activity detection\r\n");
		f_close(&file);
		return res;
	}
	thres_act = (uint16_t)acthreshold/adxl_rate;
	PRINTF("THRESHOLDACT = %u\r\n", thres_act);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TIMEACT = %u", &actime) != 1 || actime >= 0xFFFF) {
		PRINTF("Error reading time for activity detection\r\n");
		f_close(&file);
		return res;
	}
	time_act = (uint16_t)actime;
	PRINTF("TIMEACT = %u\r\n", time_act);
	
	// threshold for inactivity detection
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "THRESHOLDINACT = %u", &inacthreshold) != 1 || inacthreshold >= 0xFFFF) {
		PRINTF("Error reading threshold for inactivity detection\r\n");
		f_close(&file);
		return res;
	}
	thres_iact = (uint16_t)inacthreshold/adxl_rate;
	PRINTF("THRESHOLDINACT = %u\r\n", thres_iact);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TIMEINACT = %u", &inactime) != 1 || inactime >= 0xFFFF) {
		PRINTF("Error reading time for inactivity detection\r\n");
		f_close(&file);
		return res;
	}
	time_iact = (uint16_t)inactime;
	PRINTF("TIMEINACT = %u\r\n", time_iact);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "FIFONUM = %u", &FIFOnum) != 1 || FIFOnum >= 0xFFFF) {
		PRINTF("Error reading FIFO number\r\n");
		f_close(&file);
		return res;
	}
	FIFO_numb = (uint16_t)FIFOnum;
	PRINTF("FIFONUM = %u\r\n", FIFO_numb);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SAMPLINGTIME = %u", &time) != 1 || time >= 0xFFFF) {
		PRINTF("Error reading sampling time\r\n");
		f_close(&file);
		return res;
	}
	tstime = (uint16_t)time;
	PRINTF("SAMPLINGTIME = %u\r\n", tstime);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "SAMPLINGRATE = %u", &rate) != 1 || rate >= 0xFFFF) {
		PRINTF("Error reading sampling rate\r\n");
		f_close(&file);
		return res;
	}
	tsrate = (uint16_t)rate;
	PRINTF("SAMPLINGRATE = %u\r\n", tsrate);
	
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "NUMCHANNELS = %u", &channels) != 1 || channels < 1 || channels > 8) {
		PRINTF("Error reading channel numbers\r\n");
		f_close(&file);
		return res;
	}
	tschannels = (uint8_t)channels;
	PRINTF("NUMCHANNELS = %u\r\n", tschannels);

	f_close(&file);

	// update NAND Flash if needed
	adxlcfg.adxlrange = range;
	adxlcfg.adxlthresholdact = acthreshold;
	adxlcfg.adxltimeact = actime;
	adxlcfg.adxlthresholdinact = inacthreshold;
	adxlcfg.adxltimeinact = inactime;
	adxlcfg.adxlfifonum = FIFOnum;
	adxlcfg.adxlsamplingtime = time;
	adxlcfg.adxlsamplingrate = rate;
	adxlcfg.adxlnumchannels = channels;
	if (memcmp(&adxlcfg, adxlcfgptr, sizeof (adxlconfig_t)) != 0) {
		PRINTF("Updating NAND Flash...\r\n");
		memcpy(flashbuf, &adxlcfg, sizeof (adxlconfig_t));
		if (NandFlash_BlockErase(block) != NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash erase failed\r\n");
		} else if (NandFlash_PageProgram(flashbuf, block * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
		0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
			PRINTF("NAND Flash write success\r\n");
		} else {
			PRINTF("NAND Flash write failed\r\n");
		}
	}
	
	//  NandFlash_Init(); // no need to init
	if (NandFlash_PageRead(flashbuf1, block * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash read success\r\n");
	} else {
		PRINTF("NAND Flash read failed\r\n");
		res = FAIL;
	}
		
	sdfree(flashbuf);
	return SUCCESS;
}

static int writecfg_adxl(void)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	int res = FAIL;
	adxlconfig_t adxlcfg;
	uint8_t *flashbuf;
#ifdef GATEWAY
	uint32_t i, n;
	char ch, trigger=0, app = 0;
#endif
	
	flashbuf = (uint8_t *)sdcalloc(1, NANDFLASH_PAGE_FSIZE);
	configASSERT(flashbuf);
	
#ifdef GATEWAY
	do {
		lpc_printf(trig_menu);
		GetChar();
		do {
			ch = GetChar();
		} while (!ch);
		lpc_printf("%c\r\n", ch);
		if (ch > '0' && ch < '3') {
			trigger = ch - '0';
		} else if (ch == '3') {
			return SUCCESS;
		} else {
			lpc_printf("ERROR: bad input!\r\n");
			continue;
		}
	} while(trigger == 0);
	
	if (trigger==2)
		TrigSenIdx=0;
	else{
		TrigSenIdx=1;
	do {
		lpc_printf(params_menu);
		GetChar();
		do {
			ch = GetChar();
		} while (!ch);
		lpc_printf("%c\r\n", ch);
		if (ch > '0' && ch <= '9') {
			app = ch - '0';
		} else if (ch == '0') {
			return SUCCESS;
		} else {
			lpc_printf("ERROR: bad input!\r\n");
			continue;
		}
	} while(app == 0);
	
	if (app==1)
	{
		if (readcfg_adxl("TriggerSensing_default.cfg") != SUCCESS) {
			lpc_printf("ERROR: failed to read default configuration!\r\n");
			return FAIL;
		}
		
	}else{
		for (i = 0; i < LINELEN;) {
			ch = GetChar();
			if (!ch) {
				continue;
			}
			PutChar(ch);
			if (ch == '\n' || ch == '\r') {
				break;
			}
			if (ch == 8 && i > 0) { // backspace
				i--;
				PutChar(' ');
				PutChar(ch);
			}
			line[i++] = ch;
		}
		line[i] = '\0';
		lpc_printf("\r\n");
		if (sscanf(line, "%u", &n) != 1 || n > 0xFFFF) {
			lpc_printf("ERROR: bad input!\r\n");
			return FAIL;
		}
		
	switch (app) {
	case 2: adxl_range = (uint16_t)n; break;
	case 3: thres_act = (uint16_t)n; break;
	case 4: time_act = (uint16_t)n; break;
	case 5: thres_iact = (uint16_t)n; break;
	case 6: time_iact = (uint16_t)n; break;
	case 7: tstime = (uint16_t)n; break;
	case 8: tsrate = (uint16_t)n; break; 
	case 9: tschannels = (uint8_t)n;break;
	default:
		return SUCCESS;
	}
	}
}
#endif
	lpc_printf("- writing configuration...\r\n");

	// update xnode.cfg
	SDCard_ReWrite(TrigSenIdx);

	// update TriggerSensing.cfg in NAND Flash before SDcard, 
	adxlcfg.adxlrange = adxl_range;
	adxlcfg.adxlthresholdact = thres_act;
	adxlcfg.adxltimeact = time_act;
	adxlcfg.adxlthresholdinact = thres_iact;
	adxlcfg.adxltimeinact = time_iact;
	adxlcfg.adxlfifonum = FIFO_numb;
	adxlcfg.adxlsamplingtime = tstime;
	adxlcfg.adxlsamplingrate = tsrate;
	adxlcfg.adxlnumchannels = tschannels;
	PRINTF("Updating NAND Flash...\r\n");
	memcpy(flashbuf, &adxlcfg, sizeof (adxlconfig_t));
	if (NandFlash_BlockErase(NANDFLASH_BLOCK_TRIGGERSENSINGCFG) != NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash erase failed\r\n");
	} else if (NandFlash_PageProgram(flashbuf, NANDFLASH_BLOCK_TRIGGERSENSINGCFG * NANDFLASH_PAGE_PER_BLOCK + NANDFLASH_PAGE,
	0, NANDFLASH_RW_PAGE_SIZE) == NANDFLASH_STATUS_SUCCESS) {
		PRINTF("NAND Flash write success\r\n");
	} else {
		PRINTF("NAND Flash write failed\r\n");
	}
	
	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object 
	f_mount(0, &Fatfs);

	// go to data dir	
	ret = f_chdir("/Xnode");	
	
	if (ret) {
		PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		return res;
	}
	
	// read config file
	ret = f_open(&file, "Xnode.cfg", FA_WRITE);
	if (ret) {
		PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		return res;
	}
	
	// write data file
	// node id
	snprintf(line, LINELEN, "NODEID = %u\n", LOCAL_ADDR);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}

	snprintf(line, LINELEN, "CHANNEL = %u\n", RADIO_CHANNEL);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}

	snprintf(line, LINELEN, "POWER = %u\n", RADIO_POWER);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}
	
	// Trigger Index
	snprintf(line, LINELEN, "TRIG = %u\n", TrigSenIdx);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}
	f_close(&file);
	
	// write data file
	ret = f_open(&file, "TriggerSensing.cfg", FA_WRITE | FA_CREATE_ALWAYS);
	if (ret) {
		PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		return res;
	}
	
	// measurement range
	snprintf(line, LINELEN, "RANGE = %u\n", adxl_range);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}

	// threshold for activity detection
	snprintf(line, LINELEN, "THRESHOLDACT = %u\n", thres_act);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}
	
	// time for activity detection
	snprintf(line, LINELEN, "TIMEACT = %u\n", time_act);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}
	
	// threshold for inactivity detection
	snprintf(line, LINELEN, "THRESHOLDINACT = %u\n", thres_iact);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}
	
	// time for inactivity detection
	snprintf(line, LINELEN, "TIMEINACT = %u\n", time_iact);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}
	
	// FIFO number
	snprintf(line, LINELEN, "FIFONUM = %u\n", FIFO_numb);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}
	
	// sampling time
	snprintf(line, LINELEN, "SAMPLINGTIME = %u\n", tstime);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}

	// sampling rate
	snprintf(line, LINELEN, "SAMPLINGRATE = %u\n", tsrate);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}

	// number of channels
	snprintf(line, LINELEN, "NUMCHANNELS = %u\n", tschannels);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return res;
	}

	f_close(&file);
	vTaskDelay(1000);
	return SUCCESS;
}

void writeTask(void *parameters)
{
	writecfg_adxl();
	adxl362_setup();
	NodeReset();
	vTaskDelete(NULL);
}

static int trigsenfunc(void* arg, uint32_t len)
{
	tsmessage *ts = (tsmessage *)arg;
	int res = SUCCESS;

	if (len != sizeof (tsmessage)) {
		res = FAIL;
	}
	if (res == FAIL) {
		RemoteCommand_done(RC_CMD_TRIGSEN, FAIL, NULL, 0);
		return FAIL;
	}

	// set up configuration parameters for TriggerSensing
	lpc_printf("now setting up parameters for trigersensing\r\n");
	
	RemoteCommand_done(RC_CMD_TRIGSEN, SUCCESS, NULL, 0);
	portYIELD();
	
	tstime = ts->sampleTime;
	tsrate = ts->samplingRate;
	tschannels = ts->channelMask; // 1,2,3
	adxl_range = ts->range;
	thres_act = ts->thresholdact;
	time_act = ts->timeact;
	thres_iact = ts->thresholdinact;
	time_iact = ts->timeinact;
	FIFO_numb = ts->FIFOnumb;
	TrigSenIdx = ts->trigsen;

	lpc_printf("%u, index\r\n", TrigSenIdx);
	lpc_printf("%u, %u, %u, %u, %u, %u, %u, %u, %u\r\n", adxl_range, thres_act, time_act, thres_iact, time_iact, FIFO_numb, tstime, tsrate, tschannels);
	
	if (xTaskCreate(writeTask, "TMain", 3 * configMINIMAL_STACK_SIZE, NULL,
	(tskIDLE_PRIORITY + 1UL), (TaskHandle_t *)NULL) != pdPASS) {
		die();
	}
	
	return SUCCESS;
}

static void trigsensent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		vTaskDelay(3000);
		lpc_printf("ERROR: all nodes unresponsive\r\n");
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void trigsenesp(int success)
{
	vTaskDelay(2000);
	//NodeReset();
}

static void trigsenexec(int success, void *retval, uint32_t len)
{
	if (success != SUCCESS) {
		vTaskDelay(3000);
		lpc_printf("ERROR: all nodes unresponsive\r\n");
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

int Trig_Init(void)
{
	if (initialized) {
		return SUCCESS;
	}
	if (RemoteCommand_init() != SUCCESS
		|| RemoteCommand_register(RC_CMD_TRIGSEN, false, trigsenfunc, trigsensent, trigsenesp, trigsenexec) != SUCCESS) {
		return FAIL;
	}
	initialized = true;
	
	return SUCCESS;
}

int TrigSenSetup(void)
{
	readcfg_adxl("TriggerSensing.cfg");
	writecfg_adxl();
	return SUCCESS;
}
	
int TrigSenCfgSend(uint16_t *addrs, uint8_t addrlen)
{
	char ch;
	SDCard_Init();
	readcfg_adxl("TriggerSensing.cfg");
	
	lpc_printf("press any key before confirmation of triggering parameters\r\n");
	
	do {
	  ch = GetChar();
	} while (!ch);

	memset(&tsmsg, 0, sizeof (tsmessage));
	tsmsg.sampleTime = tstime;
	tsmsg.samplingRate = tsrate;
	tsmsg.channelMask = tschannels; // 1,2,3
	tsmsg.range = adxl_range;
	tsmsg.thresholdact = thres_act;
	tsmsg.timeact = time_act;
	tsmsg.thresholdinact = thres_iact;
	tsmsg.timeinact = time_iact;
	tsmsg.FIFOnumb = FIFO_numb;
	tsmsg.trigsen = TrigSenIdx;
	
	//TimerHandle_t tim;
	if (!initialized || !addrs || !addrlen) {
		return FAIL;
	}
	
	if (Util_Wakeup(addrs, &addrlen) != SUCCESS) {
		lpc_printf("ERROR: failed to wake up nodes; resetting...\r\n");
		return FAIL;
	}
	
	rcid = RC_CMD_TRIGSEN;
	//tim = xTimerCreate("RNTimer", pdMS_TO_TICKS(3000), pdFALSE, NULL, utiltimer);
	//configASSERT(tim);
	lpc_printf("\r\n- sending triger parameters to %u sensor nodes; wait %u seconds...\r\n", addrlen, 3);
	xTaskToNotify = xTaskGetCurrentTaskHandle();
	//xTimerStart(tim, portMAX_DELAY);
	if (RemoteCommand_execute(rcid, addrs, addrlen, &tsmsg, sizeof (tsmessage)) != SUCCESS) {
		lpc_printf("- all nodes unresponsive\r\n");
	} else {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
	//xTimerStop(tim, portMAX_DELAY);
	lpc_printf("- parameters has been sent to all nodes.\r\n");
	//xTimerDelete(tim, portMAX_DELAY);
	xTaskToNotify = NULL;
	rcid = 0xff;
	//busy = false;
	return SUCCESS;
}
#ifdef FRA  // The only difference between this one and normal adxl362_setup is that it allows GATEWAY to have sleeps too
int adxl362_setup(void)
{
	Bool half_bw = FALSE; // bandwith of the filters is 1/2 of ODR 
	Bool FIFO_ah = FALSE; // use above half of the FIFO samples
	Bool FIFO_temp = FALSE; // temperature is not included in FIFO
	
#ifndef GW_SENSE
		readcfg_adxl("TriggerSensing.cfg");
#endif	
	if(adxl362_init(TRUE)!=SUCCESS)
		PRINTF("wake up sensor initialization fails\r\n");
	adxl362_readid();
	adxl362_filter_ctl(adxl_range, half_bw, datarate);
	adxl362_FIFO_config(FIFO_ah, FIFO_temp, FIFO_mode, FIFO_numb);
	// set up interrupt pin
	adxl362_int_setup(TRUE, FALSE, 1); // INT1, Activity detection, active low
#ifndef GW_SENSE
	adxl362_int_setup(FALSE, TRUE, 2); // INT2, Inactivity detection, active high // TUFIX: Once GW_SENSE is implemented, this trigger is not really useful anymore, because it will be replaced with activity detection at startup 
#else
	adxl362_int_setup(FALSE, TRUE, 1); // INT2, Activity detection, active high //
#endif	
	adxl362_startSensing();
	TIM_Waitms(1000);
#if defined(GW_SENSE) || !defined(GATEWAY)
	adxl362_act_int(thres_act, time_act, FALSE);
	adxl362_inact_int(thres_iact, time_iact, FALSE);
#endif	
	return SUCCESS;
}
#else
int adxl362_setup(void)
{
	Bool half_bw = FALSE; // bandwith of the filters is 1/2 of ODR 
	Bool FIFO_ah = FALSE; // use above half of the FIFO samples
	Bool FIFO_temp = FALSE; // temperature is not included in FIFO
	
#ifndef GW_SENSE
		readcfg_adxl("TriggerSensing.cfg");
#endif	
	if(adxl362_init(TRUE)!=SUCCESS)
	PRINTF("wake up sensor initialization fails\r\n");
	adxl362_readid();
	adxl362_filter_ctl(adxl_range, half_bw, datarate);
	adxl362_FIFO_config(FIFO_ah, FIFO_temp, FIFO_mode, FIFO_numb);
	// set up interrupt pin
	adxl362_int_setup(TRUE, FALSE, 1); // INT1, Activity detection, active low
	adxl362_int_setup(FALSE, TRUE, 2); // INT2, Inactivity detection, active high
	adxl362_startSensing();
	TIM_Waitms(1000);
	adxl362_act_int(thres_act, time_act, FALSE);
	adxl362_inact_int(thres_iact, time_iact, FALSE);
	return SUCCESS;
}
#endif
int readadxl(int16_t *data[])
{
	uint16_t i, j;
	int16_t *FIFOData;
	double adxl_mean[3];
	FIFOData=(int16_t*)sdcalloc(FIFO_num, sizeof(int16_t));
	configASSERT(FIFOData);
	if(adxl362_init(FALSE)!=0)
		PRINTF("wake up sensor initialization fails\r\n");
	TIM_Waitms(5000);
	adxl362_FIFOread(SAMPLE_SET, FIFOData);
	memset(adxl_mean, 0, 3 * sizeof (double)); 
	for(i=0; i<SAMPLE_SET*3; i+=3)
	{
		PRINTF("%8u\t%d\t%d\t%d\n\r", (1*1000/datarate)*i/3, FIFOData[i]*adxl_rate, FIFOData[i+1]*adxl_rate, FIFOData[i+2]*adxl_rate);
	}
	for (i=0; i<SAMPLE_SET; i++)
	{
		for (j=0; j<3; j++)
		{
			if (j==2)
				data[j][i]=FIFOData[i*3+j]*adxl_rate;
			else
				data[j][i]=-FIFOData[i*3+j]*adxl_rate;
			adxl_mean[j]+=(double)data[j][i];
		}
	}
	
	for(j=0; j<3; j++)
	{
		adxl_mean[j]/=SAMPLE_SET; 
		for (i=0; i<SAMPLE_SET; i++)
			data[j][i]-=adxl_mean[j];
	}
	//adxl362_stopSensing();
	return SUCCESS;
}

void adxl_act_setup(void)
{
	if(adxl362_init(FALSE)==0)
		PRINTF("wake up sensor initialization fails\r\n");
	adxl362_disableint(TRUE);  // diable INT1
	
	adxl362_int_setup(FALSE, TRUE, 1); // INT1, activity detection
	
	ADXL362_IACTD=0;
	adxl362_clearirq();  // clear interrupt
	//adxl362_int_init();
	//adxl362_int_enable();
}

#ifdef GW_SENSE
bool adxlInt2Locked = false;
#endif

void adxl_inact_setup(void)
{
	adxl362_disableint(TRUE);  // diable INT1
	adxl362_int_setup(FALSE, TRUE, 2); // enable INT2 for Inactivity detection
#ifdef GW_SENSE
	adxlInt2Locked = true;
#endif	
	ADXL362_IACTD=0;
	adxl362_clearirq();  // clear interrupt
	//adxl362_int_init();
	//adxl362_int_enable();
}

int TriggerSensing (void)
{
	uint16_t i;
	SensorData *sd;
	ads131_init();
	ads131_on();
	readcfg_adxl("TriggerSensing.cfg");
	tschannels = rschannels;
	tsrate = rsrate;
#ifdef FRA  // indicated above, can change upper limit of sensing time by sd card
	tstime = (int) sulimit; // unit: second
#endif
	sd = setupAcquisition(RAW_DATA_RATE, tstime, 0);   // the time that train passed by the bridge all are longer than 100 seconds and shorter than 200 seconds
	for (i = 0; i < tschannels; i++) {
#ifdef STRAIN
		if (setupChannel(sd, i, tsrate, EXTERNAL) == ERROR) {
#else
		if (setupChannel(sd, i, tsrate, XNODEACCEL) == ERROR) {
#endif
			die();
			return FAIL;
		}
	}
	
	ADXL362_IACTD=0;
	adxl362_int_init();
	adxl362_int_enable();

	startAcquisition(sd, sensingFunc);
	return SUCCESS;
}

#ifndef FRA
int RTC_Reconfigure (void)
{	
	uint8_t hold; 
	uint8_t A1F, A2F;
	Clock_info *cin=sdcalloc(1,sizeof(Clock_info));
	Control_info *ctn=sdcalloc(1, sizeof(Control_info));
	configASSERT(cin && ctn);
	cin->second=0;
	cin->minute=0;
	cin->hour=1;
	cin->AMPM=FALSE;
	cin->day=7;
	cin->date=1;
	cin->month=1;
	cin->year=16;
	cin->clock24=FALSE;
	
	ctn->EOSC=TRUE; // Enable Oscillator - Bit7-0xE
	ctn->nEOSC=FALSE; // Disable Oscillator
	ctn->BBSQW=FALSE; // Battery-Backed Square-Wave Enable - Bit6-0xE
	ctn->nBBSQW=TRUE; // Battery-Backed Square-Wave Disable
	ctn->RQET=FALSE; // Convert Temperature - Bit5-0xE
	ctn->INTC=TRUE; // Interrupt Control: 0 = output square wave, 1 = output square wave when alarms hit - Bit2-0xE
	ctn->INTS= (Bool) !(ctn->INTC); // !INTC
	ctn->A1E=TRUE; // Alarm 1 Interrupt Enable - Bit0-0xE
	ctn->A2E=FALSE; // Alarm 2 Interrupt Enable - Bit1-0xE
	ctn->COSF=FALSE; // Oscillator Stop Flag - Bit7-0xF
	ctn->E32K=FALSE; // Enable 32kHz Output - Bit3-0xF
	ctn->nE32K=TRUE; // !E32K
	
	RTCS_Init();
	rtc_read(RTC_REG_STU, 1, &hold);
	A1F = (hold & 0x01);
	A2F = (hold & 0x02);	
	if(TrigSenIdx==1)
	{
		PRINTF("Set clock for 1st time \r\n");		
		RTC_SetClock(cin);
		RTC_SetAging(1);
		RTC_SetContrl(ctn);
	}	
	
	if ((!A1F)|(!A2F))
		return 0;
	else
		return 1;
}
#else
bool RTC_Reconfigure (void) // Do not reset/reinitialize clocks under any curcumstance
{	
	bool A1F, A2F;
	uint8_t hold; 
	RTCS_Init();
	rtc_read(RTC_REG_STU, 1, &hold);
	A1F = (hold & 0x01);
	A2F = (hold & 0x02);
	return (A1F | A2F);
}
#endif

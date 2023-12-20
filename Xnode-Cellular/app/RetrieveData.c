#include <xnode.h>
#include <timers.h>
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
#include "rtc.h"
#include "RetrieveData.h"
#include <lpc43xx_cgu.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_sdif.h>
#include <lpc43xx_sdmmc.h>
#include <lpc_sdmmc.h>
#include <nandflash_k9f1g08u0a.h>
#include <xnode.h>
#include <timers.h>
#include <lpc43xx_timer.h>
#include <math.h>
#include <string.h>
#include <queue.h>
#include <ads131.h>
#include <ads131_const.h>
#include <filter.h>
#include <SnoozeAlarm.h>
#include "Sensing.h"
#include "RemoteSensing.h"
#include <led.h>
#include <vcom.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ff.h"
#include <rtc.h>
#include "yaffsHEADER.h"
#include <sdcard.h>
#include <timesync.h>

#ifdef FRA
	#include "4GFTP.h"
	
	extern uint8_t counter_ftp;
#endif
#ifdef GATEWAY
	#define LINELEN               120
	static char line[LINELEN];
#endif

#define _DEBUG_               	1
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

typedef struct {
	bool latestselect;	// order to select data - start from the latest (true) or from the earliest (false)
	uint8_t datacount;
} __attribute__((packed)) drmessage;

extern uint16_t rsnodes[MAX_NODES];
uint16_t drnodes[MAX_NODES];
extern uint8_t rsncnt, rschannels;
extern uint16_t rstime, rsrate;
extern uint32_t data_index;
uint32_t datalen_tosend;
uint8_t *data_8bittype; // for NAND

drmessage drmsg;
static bool dr_initialized = false, dataready = false, spsuccess;
static uint32_t rsidx, rsok, rcid = 0xff;
static SensorData *sd;
static TaskHandle_t xTaskToNotify = NULL;
//static TimerHandle_t tim_sn = NULL;
extern bool sdready;
uint8_t rcid_copy = 0xff;
#ifndef GATEWAY
extern bool app_inactive;
#endif

extern FATFS Fatfs;
bool sensor_latestselect = true;
uint8_t sensor_datacount, gateway_datacount;
uint8_t maxchan;
uint32_t maxsize;

static void rstimer(TimerHandle_t pxTimer)
{
	RemoteCommand_stop(rcid);
	//vTaskDelay(3000);
	if (rcid == RC_CMD_DR_RETRIEVE) {
		lpc_printf("- data from node %03u is missing (timeout)\r\n", rsnodes[rsidx]);
	} else {
		lpc_printf("- command timed out\r\n");
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

#ifndef GATEWAY
/*
static void drtimer_sn(TimerHandle_t pxTimer)
{
	lpc_printf("ERROR: drtimer_sn - ReliableComm is hung; resetting\r\n");
	die();
}
*/
#else

#ifndef FRA
static const char retrieve_menu[] =
	"\r\n- Choose action:\r\n"
	"- '1'  Get last (latest) not-yet-sent dataset of all nodes\r\n"
	"- '2'  Get first (earliest) not-yet-sent dataset\r\n"
	"- '3'  Get last N (latest) not-yet-sent datasets - Define:\r\n"
	"- '4'  Get first N (earliest) not-yet-sent datasets - Define:\r\n"
	"- '5'  Return\r\n"
	"- Xnode> ";
#endif

static const char nodeselect_menu[] =
	"- Enter N: ";
Status RetrieveData_menu(void)
{
	uint32_t i, n;
	char ch, app = 0;
	memset(&drmsg, 0, sizeof (drmessage));
#ifndef FRA	// for non-FRA allow to to choose task, for FRA, task is chosen (hard-coded)
	do {
		lpc_printf(retrieve_menu);
		GetChar();
		do {
			ch = GetChar();
		} while (!ch);
		lpc_printf("%c\r\n", ch);
		if (ch > '0' && ch < '5') {
			app = ch - '0';
		} else if (ch == '5') {
			return SUCCESS;
		} else {
			lpc_printf("ERROR: bad input!\r\n");
			continue;
		}
	} while(app == 0);
#else
	app = 2;	// Get earliest set of data and start to go down to the last one
#endif
	switch (app) {
	case 1:
			drmsg.latestselect = true; //Get last (latest) not-yet-sent dataset of all nodes
			drmsg.datacount = 1;
			gateway_datacount = 1;
		break;
	case 2: // Get first (earliest) not-yet-sent dataset
			drmsg.latestselect = false; //Get last (latest) not-yet-sent dataset of all nodes
			drmsg.datacount = 1;
			gateway_datacount = 1;
		break;
	case 3: // Get last N (latest) not-yet-sent datasets - Define:
			lpc_printf(nodeselect_menu);
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
			return ERROR;
		}
		drmsg.datacount = (uint16_t)n;
		drmsg.latestselect = true; //Get last (latest) not-yet-sent dataset of all nodes
		gateway_datacount = n;
		break;
	case 4: // Get first N (earliest) not-yet-sent datasets - Define:
			lpc_printf(nodeselect_menu);
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
			return ERROR;
		}
			drmsg.datacount = (uint16_t)n;
			drmsg.latestselect = false; // Get last (latest) not-yet-sent dataset of all nodes
			gateway_datacount = n;
		break;
	default:
		break;
	}
	
#ifndef FRA	
	for (i = 0; i < gateway_datacount; i++) {
		if (i == 0) {
			if (Util_Wakeup(rsnodes, &rsncnt) != SUCCESS) { //wait 1 minute
				lpc_printf("ERROR: failed to wake up nodes; resetting...\r\n");
			}	else if (RetrieveData_start() != SUCCESS) {
				lpc_printf("ERROR: DataRetrieve start failed; resetting...\r\n");
			}
		} else {
			if (Util_Wakeup_Retrieve(rsnodes, &rsncnt) != SUCCESS) { // wait 20 seconds
				lpc_printf("ERROR: failed to wake up nodes; resetting...\r\n");
			}	else if (RetrieveData_start() != SUCCESS) {
				lpc_printf("ERROR: DataRetrieve start failed; resetting...\r\n");
			}
		}
	}
#else
	if (SyncClock_init() != SUCCESS) {
		lpc_printf("ERROR: RemoteSensing init failed; resetting...\r\n");
	} else if (SyncClock_start(false) != SUCCESS) {; // TUFIXDONE: same as format above (== success) looks better. Fix: Use status instead of int for return of SyncClock_start
		lpc_printf("ERROR: failed to read voltage\r\n");
	}	
	lpc_printf("counter_ftp = %d\n\r",counter_ftp);
	if (counter_ftp > 0)
	{
		if (RetrieveData_start() != SUCCESS) {
			lpc_printf("ERROR: DataRetrieve start failed; resetting...\r\n");
		}
	}
#endif
	lpc_printf("- done collecting data\n\r");
	return SUCCESS;
}
#endif // GATEWAY

static void getDatafromSD(void *parameterss)
{
	bool alreadysent = false;
	uint8_t fn_month, fn_date, fn_hour, fn_minute, fn_second;	// Time extracted from file name time stamp
	uint32_t timepassed ,max= 0, min = 315360000ULL;		// seconds since beginning of the year based on time stamp of file name (min = 10 years in seconds, not using full 32 bit due to "literal treated as long long")
	int f;
	struct yaffs_stat *stat;
	CHAR pathnew[121],path[200];
	struct yaffs_dev *dev;
	YCHAR *restOfPath = (YCHAR*)sdmalloc(YAFFS_MAX_NAME_LENGTH + 1); // need this malloc here or str[i] = *restOfPath; causes trouble
	struct yaffs_obj *l;
	YCHAR fn[YAFFS_MAX_NAME_LENGTH + 1],fn_select[YAFFS_MAX_NAME_LENGTH + 1], fn_select_new[YAFFS_MAX_NAME_LENGTH + 1];
	struct list_head *list_head_num;
	struct yaffs_obj *dir;

	lpc_printf("- preparing data\n\r");
	yaffs_start_up();
	yaffs_mount("/nand");

	dev = (struct yaffs_dev*) yaffsfs_FindDevice("/nand/", &restOfPath);
	stat = (struct yaffs_stat*)sdmalloc(sizeof (struct yaffs_stat));
	configASSERT(restOfPath && dev && stat);
	dir = dev->root_dir;

	PRINTF("LIST:\n\r");
	list_for_each(list_head_num, &dir->variant.dir_variant.children) {
		l = list_entry(list_head_num, struct yaffs_obj, siblings);
		yaffs_get_obj_name(l, fn, YAFFS_MAX_NAME_LENGTH + 1);
		PRINTF("----: %s \n\r", fn);
	}

	list_for_each(list_head_num, &dir->variant.dir_variant.children) {
		alreadysent = false;  // for every file, assume it's not sent yet
		l = list_entry(list_head_num, struct yaffs_obj, siblings);
		yaffs_get_obj_name(l, fn, YAFFS_MAX_NAME_LENGTH + 1);
		PRINTF("/nand/%s----", fn);
		if ((fn[strlen(fn)-8] == 's') && (fn[strlen(fn)-7] == 'e') && (fn[strlen(fn)-6] == 'n') & (fn[strlen(fn)-5] == 't')) // _sent at the end
		{
			alreadysent = true;
			PRINTF("ALREADYSENT\n\r");
		} else if ((fn[0] == 'l') && (fn[1] == 'o') && (fn[2] == 's') & (fn[3] == 't')) // lost+found 'file' or 'thing' in the list, do not want to send this
		{
			alreadysent = true;
			PRINTF("ALREADYSENT\n\r");
		}
				if(!alreadysent){
					fn_month = (fn[11]-48)*10+(fn[12]-48);
					fn_date = (fn[13]-48)*10+(fn[14]-48);
					fn_hour = (fn[16]-48)*10+(fn[17]-48);
					fn_minute = (fn[18]-48)*10+(fn[19]-48);
					fn_second = (fn[20]-48)*10+(fn[21]-48);
					timepassed = fn_second + fn_minute*60 + fn_hour*60*60 + fn_date*24*60*60 + fn_month*30*60*60*24;
					if(sensor_latestselect){
						if (timepassed>max){
							max = timepassed;
							strcpy (fn_select, fn);
						}
					} else {
						if (timepassed<min){
							min = timepassed;
							strcpy (fn_select, fn);
						}
					}
			}
	}

	// check if all files are sent by checking the max/min value
	if ((sensor_latestselect)&&(max==0)) 
	{ 
#ifdef FRA	// indicate nothing to send so the clock is set accordingly
		write_tosend(0); 
#endif
		NodeReset();
	}
	if ((!sensor_latestselect)&&(min==315360000ULL)) 
	{ 
#ifdef FRA  // indicate nothing to send so the clock is set accordingly
		write_tosend(0); 
#endif
		NodeReset();
	}

	// open and read file (make sure that it exists first, if not, then reset right above)
	snprintf(path, 256, "/nand/%s",fn_select);
	PRINTF("path = %s\n\r",path);

	f = yaffs_open(path, O_RDONLY, S_IREAD);	// we now know this file exists for sure
	yaffs_fstat(f, stat) ;
	datalen_tosend = (unsigned int) stat->st_size;
	PRINTF("stat->st_size = %d\n\r",stat->st_size);
	data_8bittype = (uint8_t *)sdcalloc(1,(unsigned int) stat->st_size);
	configASSERT(data_8bittype);
	yaffs_read(f, data_8bittype, (unsigned int) stat->st_size);
	yaffs_flush(f);
	yaffs_close(f);

	// rename the sent file	// This should be after the data is sent. But it also increases the chance node is stuck with 1 data set. Also tried but yaffs fails at yaffs_open (crash) when put in dr_resp
	strcpy(fn_select_new,fn_select);
	snprintf(fn_select_new+strlen(fn_select_new)-4,34,"_sent.txt");
	PRINTF("fn_select_new = %s, strlen(buffer) = %d\n\r",fn_select_new,strlen(fn_select_new));
	f = yaffs_open(path, O_RDONLY, S_IREAD);
	snprintf(pathnew, 256, "/nand/%s",fn_select_new);
	f = yaffs_rename(path,pathnew);
	PRINTF("f_rename = %d\n\r",f);
	if (f==-1)
	{
		NAND2SD(); // always dump all files (900 blocks) to SD, regardless, before formatting
		yaffs_format("/nand",1,1,1);
		NodeReset();
	}
	yaffs_close(f);

	lpc_printf("Setting up data completed\n\r");
	vTaskDelete(NULL);
}

static void retrievingFunc(SensorData *data)
{
	if (radio_init() != SUCCESS) {
		lpc_printf("ERROR: failed to init radio\r\n");
		die();
	}
	radio_set_short_addr(LOCAL_ADDR);
	dataready = true;
}


static int startDrPreset(void)
{
	sd = (SensorData*)sdmalloc(sizeof (SensorData));
	configASSERT(sd);
	memset(sd, 0, sizeof (SensorData));
		if (xTaskCreate(getDatafromSD, "getDatafromSD", 3 * configMINIMAL_STACK_SIZE, NULL,
		(tskIDLE_PRIORITY + 2UL), (TaskHandle_t *)NULL) != pdPASS) {
			die();
		}
	retrievingFunc(sd);
	return SUCCESS;
}

static int drpresetfunc(void* arg, uint32_t len)
{
	drmessage *dr = (drmessage *)arg;
	int res = SUCCESS;
#ifndef GATEWAY
	app_inactive = false;
#endif

	if (len != sizeof (drmessage)) {
		res = FAIL;
	}
	if (res == FAIL) {
		RemoteCommand_done(RC_CMD_DR_PRESET, FAIL, NULL, 0);
	  //xTimerStop(tim_sn, portMAX_DELAY);
	  //xTimerChangePeriod(tim_sn, pdMS_TO_TICKS(30000), portMAX_DELAY);
		//xTimerGenericCommandportYIELD();
		return FAIL;
	}

	RemoteCommand_done(RC_CMD_DR_PRESET, SUCCESS, NULL, 0);
	portYIELD();
	radio_sleep();

	sensor_latestselect = (bool) dr->latestselect;
	sensor_datacount = (uint8_t) dr->datacount;

  startDrPreset();
	return SUCCESS;
}

static void drpresetsent(uint16_t *targets, uint8_t tcount)	// must be modified for cases with next data batch to be sent
{
	if (tcount == 0) {
		//vTaskDelay(3000);
		lpc_printf("ERROR: all nodes unresponsive\r\n");
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void drpresetresp(int success)
{
	if (success != SUCCESS) {
		lpc_printf("ERROR: failed to set parameters\r\n");
		die();
	}
}

static void drpresetexec(int success, void *retval, uint32_t len)
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

static int drretrievefunc(void* arg, uint32_t len)	// same as getdatafunc, but keep separated for future modifications
{
	int res = SUCCESS;

//	xTimerStop(tim_sn, portMAX_DELAY);
//	xTimerChangePeriod(tim_sn, 5*60000, portMAX_DELAY);
	if (!dataready || !sd) {
		res = FAIL;
	}

	lpc_printf("- drretrievefunc sending back data (%u bytes)...\r\n", datalen_tosend);
	RemoteCommand_done(RC_CMD_DR_RETRIEVE, res, data_8bittype, datalen_tosend);
	portYIELD();
	return res;
}

static void drretrievesent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		//vTaskDelay(5000);
		lpc_printf("- data from node %03u is missing (timeout)\r\n", rsnodes[rsidx]);
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void drretrieveresp(int success)
{
	LED_Off();
	NodeReset();
//	SnoozeAlarm_Sleep_Retrieve(); // wake up after 20 seconds
}

uint32_t RetrieveunpackSensorData(uint8_t *data, SensorData **sd, uint32_t *max) // get data package information (rate,  # of channels, size) before unpack it
{
	int i;
	uint32_t sdidx, len;

	configASSERT(data && sd && max);

	len = *(uint32_t *)data;
	*max = *(uint32_t *)(data + len - sizeof (uint32_t) - sizeof(uint64_t));
	*sd = (SensorData *)sdcalloc(len - 2 * sizeof (uint32_t) - sizeof(uint64_t), 1);
	configASSERT(*sd);
	memcpy(*sd, data + sizeof(uint32_t), len - 2 * sizeof (uint32_t) - sizeof(uint64_t));

	sdidx = sizeof (SensorData);
	for (i = 0; i < 8; i++)
	{
		if(sdidx > len) { break;}		// go for max number of channels and break out later
		if ((*sd)->channels[i].sampSize == 0) {
			break; // assume that the channels are continous (like 0,1,2 , not 0,1,2,4)
		}
		(*sd)->channels[i].sampData = (float *)((uint8_t *)*sd + sdidx);
		sdidx += (*sd)->channels[i].sampSize * sizeof (float);
	}
	rschannels = i;
	return (*sd)->channels[0].sampSize;
}

#ifndef FRA	// FRA code just save data to NAND and SD, no more further processing
static void processdatatask(void *pvParameters)
{
#ifdef PRINTDATA
	uint32_t i, j;
#endif
	//uint32_t max = (uint32_t)pvParameters;
	(void)pvParameters;
	lpc_printf("- node %03u: battery voltage is %.2fV, charging current is %.2fmA\n\r", rsnodes[rsidx], sd->voltage, sd->current);
	lpc_printf("- writing data to SD card...");
  if (RemoteSensing_writedata(sd, rsnodes[rsidx], 0) != SUCCESS) {
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
	lpc_printf("\r\n");
#endif

	sdfree(sd);
	sd = NULL;
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
		//portYIELD();
	}
	vTaskDelete(NULL);
}

static void Retrieveprocessdata(uint8_t *data, uint32_t len)
{
	uint32_t max;

	len = RetrieveunpackSensorData(data, &sd, &max);
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

static void drretrieveexec(int success, void *retval, uint32_t len)
{
	if (success != SUCCESS) {
		//vTaskDelay(3000);
		lpc_printf("- data from node %03u is missing (no response)\r\n", rsnodes[rsidx]);
	} else {
		rsok++;
		lpc_printf("- data received from node %03u\r\n", rsnodes[rsidx]);
		Retrieveprocessdata((uint8_t *)retval, len);
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
		portYIELD();
	}
}
#else // FRA way seems to be shorter and more straight-forward (TUCONSIDER) - BUT the written data is not processed
#ifdef GW_SENSE			
extern bool adxlInt2Locked;
#endif
static void drretrieveexec(int success, void *retval, uint32_t len)
{
	if (success != SUCCESS) {
		//vTaskDelay(3000);
		lpc_printf("- data from node %03u is missing (no response)\r\n", rsnodes[rsidx]);
	} else {
		rsok++;
		lpc_printf("- data received from node %03u\r\n", rsnodes[rsidx]);
#ifdef GW_SENSE
		adxlInt2Locked = true;
#endif		
	// Write to NAND flash
		WriteToNAND((uint8_t *)retval , &len);
	
	// Write to sd card
		WriteToSD((uint8_t *)retval , &len);
	
	// update ftp file counter
		write_isFTP();
#ifdef GW_SENSE
		adxlInt2Locked = false;
#endif
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
		portYIELD();
	}
}

#endif
int RetrieveData_init(void)
{
	if (dr_initialized) {
		PRINTF("Already initialized RetrieveData\n\r");
		return SUCCESS;
	}
  if (RemoteCommand_init() != SUCCESS
	|| RemoteCommand_register(RC_CMD_DR_PRESET, false, drpresetfunc, drpresetsent, drpresetresp, drpresetexec) != SUCCESS
	|| RemoteCommand_register(RC_CMD_DR_RETRIEVE, true, drretrievefunc, drretrievesent, drretrieveresp, drretrieveexec) != SUCCESS) {
		return FAIL;
	}
	
	dr_initialized = true;
	return SUCCESS;
}

int RetrieveData_start(void)
{
  TimerHandle_t tim;
	if (!dr_initialized) {
		return FAIL;
	}
	tim = xTimerCreate("RSTimer", pdMS_TO_TICKS(30000), pdFALSE, NULL, rstimer);
	configASSERT(tim);
	lpc_printf("\r\n- sending command to sensor node(s); wait %u seconds...\r\n", 3);
	spsuccess = false;
	rcid = RC_CMD_DR_PRESET;
  xTaskToNotify = xTaskGetCurrentTaskHandle();
	xTimerStart(tim, portMAX_DELAY);

	if (RemoteCommand_execute(RC_CMD_DR_PRESET, rsnodes, rsncnt, &drmsg, sizeof (drmessage)) != SUCCESS) {
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

	lpc_printf("- retrieving parameters sent to all node(s).\r\n");
	vTaskDelay(5000);
	lpc_printf("\r\n- retrieving data from sensor node(s)...\r\n");
	rsidx = rsok = 0;
	rcid = RC_CMD_DR_RETRIEVE;
	rcid_copy = RC_CMD_DR_RETRIEVE; // this is for the external variable to be used in SendLData to stop the timers during receiving data
	for (rsidx = 0, rsok = 0; rsidx < rsncnt; ++rsidx) {
		lpc_printf("\r\n- requesting data from node %03u...\r\n", rsnodes[rsidx]);
		xTimerChangePeriod(tim, 120000, portMAX_DELAY); // for 8 channels, 300 sec take longer - need a function for this wait time to optimize it
		if (RemoteCommand_execute(RC_CMD_DR_RETRIEVE, rsnodes + rsidx, 1, NULL, 0) != SUCCESS) {
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

	if (rsok == 0) {
		lpc_printf("ERROR: all nodes unresponsive; RemoteSensing failed!\r\n");
	} else {
		lpc_printf("- data retrieved from %u nodes\r\n", rsok);
	}

	lpc_printf("- resetting...\r\n");
	vTaskDelay(500);
	return (rsok == 0);
}


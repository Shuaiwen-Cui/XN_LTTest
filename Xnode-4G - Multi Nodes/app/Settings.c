#include <xnode.h>
#include <timers.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <ff.h>
#include <RemoteCommand.h>
#include <Utils.h>
#include "RemoteSensing.h"
#include "Settings.h"
#include "TriggerSensing.h"
#include "sdcard.h"

#define LINELEN               120

extern FATFS Fatfs;
static char line[LINELEN];

typedef struct {
	uint8_t channel;
	uint8_t power;
	uint8_t trig;
} __attribute__((packed)) xnodecfgmessage;

static xnodecfgmessage xcmsg;
uint8_t xcchannel, xcpower, xctrig;
uint8_t NewLocalNodeID, NewLocalChannel, NewLocalPower, NewLocalTrigSenIdx;
uint8_t leaf_xcchannel, leaf_xcpower, leaf_xctrig;
extern uint8_t rsidx;
extern uint8_t rsok;
extern uint8_t rcid;
extern uint8_t TrigSenIdx;
extern TaskHandle_t xTaskToNotify;

int doreadcfg(const char *fn);

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

static void xctimer(TimerHandle_t pxTimer)
{
	RemoteCommand_stop(rcid);
	//vTaskDelay(100);
	if (rcid == RC_CMD_XNODECFG) {
		lpc_printf("- node %03u unresponsive\r\n", rsnodes[rsidx]);
	} else {
		lpc_printf("- command timed out\r\n");
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

#ifdef GATEWAY
static const char settings_menu[] =
	"\r\n- Choose action:\r\n"
	"- '1'  Change sensing settings\r\n"
	"- '2'  Change event trigger settings\r\n"
	"- '3'  Change communication settings\r\n"
	"- '4'  Return\r\n"
	"- Xnode> ";

static const char params_menu[] =
	"\r\n- Choose action:\r\n"
	"- '1'  Restore defaults\r\n"
	"- '2'  Change sampling time\r\n"
	"- '3'  Change sampling rate\r\n"
	"- '4'  Change node list\r\n"
	"- '5'  Return\r\n"
	"- Xnode> ";

static const char st_menu[] =
	"- Enter sampling time (seconds): ";

static const char sr_menu[] = 
	"- Enter sampling rate (Hz): ";

static const char nl_menu[] = 
	"- Enter node list (e.g.: 1,2,3): ";

static const char XnodeCfg_menu[] =
	"- Choose location to be changed:\r\n"
	"- '1'  Local (gateway)\r\n"
	"- '2'  Remote (sensor)\r\n"
	"- Xnode> ";

static const char XnodeCfg_Localmenu[] =
	"\r\n- Choose action:\r\n"
	"- '1'  Change local radio channel\r\n"
	"- '2'  Change local radio power\r\n"
	"- '3'  Return\r\n"
	"- Xnode> ";

static const char XnodeCfg_Remotemenu[] =
	"\r\n- Choose action:\r\n"
	"- '1'  Change remote radio channel\r\n"
	"- '2'  Change remote radio power\r\n"
	"- '3'  Return\r\n"
	"- Xnode> ";

static const char XnodeCfg_LocalChannel_menu[] =
	"- NOTE: range 11-26 with recommended values: 15, 20, 25, 26\r\n"
	"- Enter channel: ";

static const char XnodeCfg_LocalPower_menu[] =
	"- NOTE: range 0-15, inverse scale: 0 = max, 15 = min\r\n"
	"- Enter power: ";

#ifdef FRA	// menu for FRA tasks, but not really necessary
#include "4gFTP.h"

static const char fra_menu[] =
	"\r\n- Choose action:\r\n"
	"- '1'  Check status of all sensor nodes\r\n" // current, voltage, andm more
	"- '2'  Synchronize RTC with network time\r\n" // consider making this more robust to get time from network
	"- '3'  Collect and upload data to FTF server\r\n" //consider doing upload in 1 shot for multiple sets of 1 node
	"- '4'  Download new configuration from the FTP site\r\n" 
	"- '5'  Return\r\n"
	"- Xnode> ";
#endif

void XnodeCfg_settings(void)
{
	char xcch;
	lpc_printf(XnodeCfg_menu);
	do {
		xcch = GetChar();
	} while (!xcch);
	lpc_printf("%c\r\n", xcch);
	switch (xcch) {
		case '1': // Local
			if (XnodeCfg_setparams() != SUCCESS) {
				lpc_printf("ERROR: failed to set set Xnode configuration parameters locally\r\n");
			}
			break;
		case '2': //Remote
			if (XnodeCfgSend_init() != SUCCESS) {
				lpc_printf("ERROR: failed to initialize\r\n");
			}
			if (XnodeCfgSend(rsnodes, rsncnt) != SUCCESS) {
				lpc_printf("ERROR: failed to set send\r\n");
			}
			break;
		default:
			lpc_printf("ERROR: bad input!\r\n");
	}
}

void Settings_menu(void)
{
	char ch, app = 0;

	do {
		lpc_printf(settings_menu);
		GetChar();
		do {
			ch = GetChar();
		} while (!ch);
		lpc_printf("%c\r\n", ch);
		if (ch > '0' && ch < '5') {
			app = ch - '0';
		} else if (ch == '4') {
			return;
		} else {
			lpc_printf("ERROR: bad input!\r\n");
			continue;
		}
	} while(app == 0);

	switch (app) {
	case 1: // Sensing params
		RemoteSensing_setparams();
		break;
	case 2: // Trigger params
		if (TrigSenSetup() != SUCCESS) {
			lpc_printf("ERROR: failed to set trigger configuration remotely\r\n");
		}
		break;
	case 3: // Comm params
		XnodeCfg_settings();
		break;
	default:
		break;
	}
}

int RemoteSensing_setparams(void)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	uint32_t i, j, n;
	char *nline, *str, ch, app = 0;

	do {
		lpc_printf(params_menu);
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

	switch (app) {
	case 1: // defaults
		if (doreadcfg("RemoteSensing_default.cfg") != SUCCESS) {
			lpc_printf("ERROR: failed to read default configuration!\r\n");
			return FAIL;
		}
		break;
	case 2: // sampling time
		lpc_printf(st_menu);
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
		rstime = (uint16_t)n;
		break;
	case 3: // sampling rate
		lpc_printf(sr_menu);
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
		if (n != 500 && n != 200 && n != 100 && n != 50 && n != 20 && n != 10) {
			lpc_printf("ERROR: supported sampling rates are: 10, 20, 50, 100, 200, and 500Hz\r\n");
			return FAIL;
		}
		rsrate = (uint16_t)n;
		break;
	case 4: // node list
		nline = sdmalloc(512);
		configASSERT(nline);
		lpc_printf(nl_menu);
		for (i = 0; i < 511;) {
			ch = GetChar();
			if (!ch) {
				continue;
			}
			if (ch == '\n' || ch == '\r') {
				break;
			}
			if (ch == 8 && i > 0) { // backspace
				i--;
				PutChar(ch);
				PutChar(' ');
				PutChar(ch);
			} else if ((ch >= '0' && ch <= '9') || ch == ',' || ch == ' ') {
				nline[i++] = ch;
				PutChar(ch);
			}
		}
		nline[i] = '\0';
		lpc_printf("\r\n");

		// parse nodeids
		str = strtok(nline, " ,\r\n");
		for (i = 0; i < MAX_NODES && str != NULL; ++i, str = strtok(NULL, " ,\r\n")) {
			if (sscanf(str, "%u", &n) != 1 || n < 1 || n >= 0xFFFF) {
				break;
			}
			rsnodes[i] = (uint16_t)n;
		}

		sdfree(nline);
		rsncnt = i;
		if (rsncnt == 0) {
			lpc_printf("ERROR: bad input!\r\n");
			return FAIL;
		}
		break;
	default:
		return SUCCESS;
	}

	lpc_printf("- writing configuration...\r\n");

	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object
	f_mount(0, &Fatfs);

	// go to data dir
	ret = f_chdir("/Xnode");
	if (ret) {
		#if _DEBUG
			PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// write data file
	ret = f_open(&file, "RemoteSensing.cfg", FA_WRITE | FA_CREATE_ALWAYS);
	if (ret) {
		#if _DEBUG_
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// sampling time
	snprintf(line, LINELEN, "SAMPLINGTIME = %u\n", rstime);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// sampling rate
	snprintf(line, LINELEN, "SAMPLINGRATE = %u\n", rsrate);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// number of channels
	snprintf(line, LINELEN, "NUMCHANNELS = %u\n", rschannels);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// number of nodes
	snprintf(line, LINELEN, "NUMNODES = %u\n", rsncnt);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// node ids
	nline = sdmalloc(512);
	configASSERT(nline);
	n = snprintf(nline, 512, "NODEIDS = ");

	for (i = 0, j = n; i < rsncnt; ++i, j += n) {
		n = snprintf(nline + j, 512 - j, "%u,", rsnodes[i]);
	}
	snprintf(nline + j - 1, 512 - j + 1, "\n");
	if (f_puts(nline, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}
	sdfree(nline);

	f_close(&file);
	lpc_printf("- resetting...\r\n");
	vTaskDelay(1000);
	return SUCCESS;
}

int XnodeCfg_setparams(void)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object
	uint32_t i, n;
	char ch, app = 0;

	// Get these items so if not assigned they don't write junk to the sd card
	NewLocalNodeID = LOCAL_ADDR;
	NewLocalChannel = RADIO_CHANNEL;
	NewLocalPower = RADIO_POWER;
	NewLocalTrigSenIdx = TrigSenIdx;
	do {
		lpc_printf(XnodeCfg_Localmenu);
		GetChar();
		do {
			ch = GetChar();
		} while (!ch);
		lpc_printf("%c\r\n", ch);
		if (ch > '0' && ch < '3') {
			app = ch - '0';
		} else if (ch == '3') {
			return SUCCESS;
		} else {
			lpc_printf("ERROR: bad input!\r\n");
			continue;
		}
	} while(app == 0);

	switch (app) {
	case 1: // Local channel
		lpc_printf(XnodeCfg_LocalChannel_menu);
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
		if (n < 11 && n > 26) {
			lpc_printf("ERROR: supported radio channels are in the range 11 to 26\r\n");
			return FAIL;
		}
		NewLocalChannel = (uint16_t)n;
		break;
	case 2: // Local power
	lpc_printf(XnodeCfg_LocalPower_menu);
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
		if (sscanf(line, "%u", &n) != 1 || n > 0xFF) {
			lpc_printf("ERROR: bad input!\r\n");
			return FAIL;
		}
		if (n > 15) { //unsigned so always >0
			lpc_printf("ERROR: supported radio power levels are in the range 0 to 15\r\n");
			return FAIL;
		}
		NewLocalPower = (uint16_t)n;
		break;
	default:
		return SUCCESS;
	}

	lpc_printf("- writing configuration...\r\n");

	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object
	f_mount(0, &Fatfs);

	// go to data dir
	ret = f_chdir("/Xnode");
	if (ret) {
		#if _DEBUG_
			PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// write data file
	ret = f_open(&file, "Xnode.cfg", FA_WRITE | FA_CREATE_ALWAYS);
	if (ret) {
		#if _DEBUG_
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// sampling time
	snprintf(line, LINELEN, "NODEID = %u\n", NewLocalNodeID);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// sampling rate
	snprintf(line, LINELEN, "CHANNEL = %u\n", NewLocalChannel);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// number of channels
	snprintf(line, LINELEN, "POWER = %u\n", NewLocalPower);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	// number of nodes
	snprintf(line, LINELEN, "TRIG = %u\n", NewLocalTrigSenIdx);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}

	f_close(&file);
	lpc_printf("- resetting...\r\n"); // Then the NAND can be updated in the next boot up

	vTaskDelay(200);
	return SUCCESS;
}
#endif

static int xcwritefunc(void)
{
	FRESULT ret;              // Result code
	FIL file;                 // File object

	leaf_xcchannel = (leaf_xcchannel == 0xFF) ? RADIO_CHANNEL: leaf_xcchannel;
	leaf_xcpower = (leaf_xcpower == 0xFF) ? RADIO_POWER: leaf_xcpower;
	leaf_xctrig = (leaf_xctrig == 0xFF) ? TrigSenIdx: leaf_xctrig;

	lpc_printf("- writing configration parameters to NAND flash\r\n");
	SDCard_ReWriteALL(LOCAL_ADDR, leaf_xcchannel, leaf_xcpower, leaf_xctrig);
// Writing to Xnode.cfg
	lpc_printf("- writing configration parameters to SD card\r\n");

	// mount sdcard
	memset(&Fatfs, 0, sizeof(FATFS));  // Clear file system object
	f_mount(0, &Fatfs);
	// go to data dir
	ret = f_chdir("/Xnode");

	if (ret) {
		#if _DEBUG_
			lpc_printf("Change directory error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// write data file
	ret = f_open(&file, "Xnode.cfg", FA_WRITE | FA_CREATE_ALWAYS);

	if (ret) {
		#if _DEBUG_
			PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		#endif
		return FAIL;
	}

	// sampling time
	snprintf(line, LINELEN, "NODEID = %u\n", LOCAL_ADDR); // not changed
	PRINTF("- %s\r",line);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}
	// sampling rate

	snprintf(line, LINELEN, "CHANNEL = %u\n", leaf_xcchannel);
	PRINTF("- %s\r",line);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}
	// number of channels
	snprintf(line, LINELEN, "POWER = %u\n", leaf_xcpower);
	PRINTF("- %s\r",line);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}
	// number of nodes
	snprintf(line, LINELEN, "TRIG = %u\n", leaf_xctrig);
	PRINTF("- %s\r",line);
	if (f_puts(line, &file) == 0) {
		PRINTF("Error writing data\r\n");
		f_close(&file);
		return FAIL;
	}
	f_close(&file);

	return SUCCESS;
}

void XCWriteTask(void *parameters)
{
	xcwritefunc();
	NodeReset();
	vTaskDelete(NULL);
}

static int xnodecfgfunc(void* arg, uint32_t len)
{
	int res = SUCCESS;
	xnodecfgmessage *xc = (xnodecfgmessage *)arg;

	if (len != sizeof (xnodecfgmessage)) {
		res = FAIL;
	}
	if (res == FAIL) {
		RemoteCommand_done(RC_CMD_XNODECFG, FAIL, NULL, 0);
		return FAIL;
	}

	// set up configuration parameters for XnodeCfg
	lpc_printf("- setting up configration parameters\r\n");

	leaf_xcchannel = xc->channel; PRINTF("- received channel: %d\n\r",leaf_xcchannel);
	leaf_xcpower = xc->power; PRINTF("- received power: %d\n\r",leaf_xcpower);
	leaf_xctrig = xc->trig;  PRINTF("- received trig: %d\n\r",leaf_xctrig);

	if (xTaskCreate(XCWriteTask, "XCWrite", 3 * configMINIMAL_STACK_SIZE, NULL,
	(tskIDLE_PRIORITY + 2UL), (TaskHandle_t *)NULL) != pdPASS) {
		die();
	}

	RemoteCommand_done(RC_CMD_XNODECFG, SUCCESS, NULL, 0);
	portYIELD();

	return SUCCESS;
}

static void xnodecfgsent(uint16_t *targets, uint8_t tcount)
{
	if (tcount == 0) {
		//vTaskDelay(1000);
		lpc_printf("- No response from node %03u\r\n", rsnodes[rsidx]);
		if (xTaskToNotify) {
			xTaskNotifyGive(xTaskToNotify);
		}
	}
}

static void xnodecfgresp(int success)
{
	LED_Off();
	//vTaskDelay(2000);
	NodeReset();
}

static void xnodecfgexec(int success, void *retval, uint32_t len)
{
	if (success != SUCCESS) {
		//vTaskDelay(3000);
		lpc_printf("ERROR: all nodes unresponsive\r\n");
	}
	if (xTaskToNotify) {
		xTaskNotifyGive(xTaskToNotify);
	}
}

int XnodeConfig_Init(void)
{
	if (RemoteCommand_init() != SUCCESS
	|| RemoteCommand_register(RC_CMD_XNODECFG, false, xnodecfgfunc, xnodecfgsent, xnodecfgresp, xnodecfgexec) != SUCCESS) {
		return FAIL;
	}
	return SUCCESS;
}

int XnodeCfgSend_init(void)
{
#ifdef GATEWAY
	char ch;
	uint8_t i, app = 0;
	uint8_t n;
#endif


#ifdef GATEWAY
	// On sensor node, if 0xFF is received, the value won't be changed
	xcchannel = 0xFF;
	xcpower = 0xFF;
	xctrig = 0xFF;
	do {
		lpc_printf(XnodeCfg_Remotemenu);
		GetChar();
		do {
			ch = GetChar();
		} while (!ch);
		lpc_printf("%c\r\n", ch);
		if (ch > '0' && ch < '3') {
			app = ch - '0';
		} else if (ch == '3') {
			return SUCCESS;
		} else {
			lpc_printf("ERROR: bad input!\r\n");
			continue;
		}
	} while(app == 0);

	switch (app) {
	case 1: // Remote channel
		lpc_printf(XnodeCfg_LocalChannel_menu);
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
		if (sscanf(line, "%" SCNu8, &n) != 1 || n > 0xFF) {
			lpc_printf("ERROR: bad input!\r\n");
			return FAIL;
		}
		if (n < 11 && n > 26) {
			lpc_printf("ERROR: supported radio channels are in the range 11 to 26\r\n");
			return FAIL;
		}
		xcchannel = (uint8_t)n;
		break;
	case 2: // Remote power
	lpc_printf(XnodeCfg_LocalPower_menu);
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
		if (sscanf(line, "%" SCNu8, &n) != 1 || n > 0xFF) {
			lpc_printf("ERROR: bad input!\r\n");
			return FAIL;
		}
		if (n > 15) { // unsigned so can't be smaller than 0
			lpc_printf("ERROR: supported radio power levels are in the range 0 to 15\r\n");
			return FAIL;
		}
		xcpower = (uint8_t)n;
		break;
	default:
		return SUCCESS;
	}
#else // better confirm these numbers
	// defaults
	xcchannel = 26;
	xcpower = 15;
	xctrig = 0;
#endif

	return SUCCESS;
}

int XnodeCfgSend(uint16_t *addrs, uint8_t addrlen)
{
	TimerHandle_t tim;

	if (!addrs || !addrlen) {
		return FAIL;
	}

	if (Util_Wakeup(addrs, &addrlen) != SUCCESS) {
		lpc_printf("ERROR: failed to wake up nodes; resetting...\r\n");
		return FAIL;
	}

	memset(&xcmsg, 0, sizeof (xnodecfgmessage));
	xcmsg.channel = xcchannel;
	xcmsg.power = xcpower;
	xcmsg.trig = xctrig;
	rcid = RC_CMD_XNODECFG;
	lpc_printf("\r\n- sending Xnode configuration parameters to %u sensor nodes; wait %u seconds...\r\n", addrlen, 3);
	xTaskToNotify = xTaskGetCurrentTaskHandle();
	tim = xTimerCreate("XCTimer", pdMS_TO_TICKS(30000), pdFALSE, NULL, xctimer);
	configASSERT(tim);

	for (rsidx = 0, rsok = 0; rsidx < rsncnt; ++rsidx) {
		lpc_printf("\r\n- sending parameters to node %03u...\r\n", rsnodes[rsidx]);
		xTimerChangePeriod(tim, 1000, portMAX_DELAY);
		if (RemoteCommand_execute(rcid, rsnodes + rsidx, 1, &xcmsg, sizeof (xnodecfgmessage)) != SUCCESS) {
			lpc_printf("- data from node %03u is missing (command failed)\r\n", rsnodes[rsidx]);
		} else {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
		xTimerStop(tim, portMAX_DELAY);
		LED_Off();
		vTaskDelay(100);
	}

	lpc_printf("- parameters have been sent to all nodes.\r\n");
	xTaskToNotify = NULL;
	rcid = 0xff;
	vTaskDelay(200);
	return SUCCESS;
}

#ifdef FRA	// Sub-tasks of main task #8, for FRA purposes
#ifdef GATEWAY
#include "4gFTP.h"
#include "yaffsHEADER.h"
#include "SnoozeAlarm.h"
int RetrieveData_menu(void);
int SyncClock_init(void);
int FRAUtil_ReadVoltage(uint16_t *addrs, uint8_t addrlen);
Status SyncClock_start(bool use4G);

extern bool isformat;
char FRAch;
#ifdef GW_SENSE			
extern bool adxlInt2Locked;
#endif
void FRA_menu(void)
{
	uint32_t count;
	lpc_printf(fra_menu);
	lpc_printf("Task number selected: %c\r\n", FRAch);
	// do minimal out here (before switch), so node can go back to sleep faster
	switch (FRAch) {
		case '1': // Check status
			CheckBatteryVoltage();
			if (FRAUtil_ReadVoltage(rsnodes, rsncnt) != SUCCESS) {
				lpc_printf("ERROR: failed to read voltage\r\n");
			}
			break;
		case '2': // Synchronize RTC // TUFIX: Make this same as status asking: Current clock, voltage, current, #of files, %of file not sent, %of files in sd card, TEMPERATURE in ADC
			write_istherefiletosend(1); // Twice a day, faking data set to be sent to make sure anything stuck is sent out, important
			CheckBatteryVoltage();
			if (SyncClock_init() != SUCCESS) {
				lpc_printf("ERROR: RemoteSensing init failed; resetting...\r\n");
			}
			if (SyncClock_start(true) != SUCCESS) {; // TUFIXDONE: same as format above (== success) looks better. Fix: Use status instead of int for return of SyncClock_start
				lpc_printf("ERROR: failed to read voltage\r\n");
			}
			break;
		case '3': // Collect data from sensors & Upload
			count = read_isFTP();	 
			if (count>DATACOUNTHRESHOLD)
			{
				CheckBatteryVoltage();  // check for Gateway so that have enough energy for FTP transmission - EDIT: This version doesn't care much about this, let it try, if it fails it will stop
				// Prepare yaffs later to prevent delay of sending commands in the beginning. Don't do this more than once
#ifdef GW_SENSE			
				adxlInt2Locked = true; // critical memory access section
#endif
				yaffs_start_up();
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
#ifdef GW_SENSE
				adxlInt2Locked = false; // critical memory access section
#endif
				ManualFTP();
				NodeReset();
			}
			if (RetrieveData_menu()!= SUCCESS) { // TUFIXDONE: add output (fail, success) so looks for uniform. Fix: Use Status
				lpc_printf("Retrieved zero set...\r\n");
				NodeReset();
			}
			break;
		case '4': // Download Configuration file from the FTP site
			UpdateParamMQTT();
			NodeReset();
			break;
		case '5': // Return/Reset
			NodeReset();
			break;
		default:
			lpc_printf("ERROR: bad input!\r\n");
	}
}
#endif
#endif

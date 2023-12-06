#ifdef GATEWAY
#include <xnode.h>
#include <stdio.h>
#include <string.h>
#include <ff.h>
#include "AutoMonitor.h"
#include "RemoteSensing.h"

uint8_t AM_HOURS = 0;

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

#define LINELEN               80
extern FATFS Fatfs;
static char line[LINELEN];

static const char am_menu[] =
	"- Choose period (hours):\r\n"
	"- '1'  12\r\n"
	"- '2'  24\r\n"
	"- '3'  48\r\n"
	"- Xnode> ";

int AutoMonitor_running(void)
{
	uint32_t i;
	if (!AM_HOURS) {
		return 0;
	}
	lpc_printf("- waiting for AutoMonitor; press 'x' to exit...\r\n");
	for (i = 0; i < AM_HOURS * 3600 - 1800; ++i) {
		vTaskDelay(1000);
		if (GetChar() == 'x') {
			lpc_printf("- stopping AutoMonitor...\r\n");
			AM_HOURS = 0;
			if (AutoMonitor_writecfg() != SUCCESS) {
				lpc_printf("ERROR: failed to read AutoMonitor config\r\n");
			}
			vTaskDelay(200);
			NodeReset();
		}
	}
	lpc_printf("- AutoMonitor run starting...\r\n");
	RemoteSensing_menu();
	return 1;
}

int AutoMonitor_menu(void)
{
	char ch;
	AM_HOURS = 0;
	do {
		lpc_printf(am_menu);
		do {
			ch = GetChar();
		} while (!ch);
		lpc_printf("%c\r\n", ch);
		switch (ch) {
		case '1':
			AM_HOURS = 12;
			break;
		case '2':
			AM_HOURS = 24;
			break;
		case '3':
			AM_HOURS = 48;
			break;
		default:
			lpc_printf("ERROR: bad input!\r\n");
			continue;
		}
	} while(AM_HOURS == 0);
	AutoMonitor_writecfg();
	lpc_printf("- starting AutoMonitor with a period of %u hours...\r\n", AM_HOURS);
	RemoteSensing_menu();
	return 0;
}

int AutoMonitor_readcfg(void)
{
	FRESULT ret;              // Result code
  FIL file;                 // File object
	uint32_t automon;

  // go to config dir	
	ret = f_chdir("/Xnode");
  if (ret) {
    PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		return FAIL;
	}

	// read config file
  ret = f_open(&file, "AutoMonitor.cfg", FA_READ);
  if (ret) {
    PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		return FAIL;
	}

	// AutoMonitor config
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "AUTOMONITOR = %u", &automon) != 1 || automon >= 0xFF) {
    PRINTF("Error reading AutoMonitor config\r\n");
		f_close(&file);
		return FAIL;
	}
	AM_HOURS = (uint16_t)automon;
	PRINTF("AUTOMONITOR = %u\r\n", AM_HOURS);

	f_close(&file);
	return SUCCESS;
}

int AutoMonitor_writecfg(void)
{
	FRESULT ret;              // Result code
  FIL file;                 // File object

  // go to config dir	
	ret = f_chdir("/Xnode");
  if (ret) {
    PRINTF("Change directory error %u: %s\r\n", ret, print_error(ret));
		return FAIL;
	}

	// write config file
  ret = f_open(&file, "AutoMonitor.cfg", FA_WRITE | FA_CREATE_ALWAYS);
  if (ret) {
    PRINTF("Open file error %u: %s\r\n", ret, print_error(ret));
		return FAIL;
	}

	// AutoMonitor config
	snprintf(line, LINELEN, "AUTOMONITOR = %u\r\n", AM_HOURS);
	if (f_puts(line, &file) == 0) {
    PRINTF("Error writing AutoMonitor config\r\n");
		f_close(&file);
		return FAIL;
	}

	f_close(&file);
	return SUCCESS;
}
#endif

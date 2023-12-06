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


typedef struct {
	uint16_t nodeid;
	uint8_t channel;
	uint8_t power;
	uint8_t trigsen;
} flashconfig_t;

bool sdready = false;

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
	if (f_gets(line, LINELEN, &file) == NULL || sscanf(line, "TRIG = %u", &idx) != 1 || idx > 2) {
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

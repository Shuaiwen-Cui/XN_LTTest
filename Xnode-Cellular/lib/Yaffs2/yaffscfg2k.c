/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2011 Aleph One Ltd.
 *   for Toby Churchill Ltd and Brightstar Engineering
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * yaffscfg2k.c  The configuration for the "direct" use of yaffs.
 *
 * This file is intended to be modified to your requirements.
 * There is no need to redistribute this file.
 */

#include "yaffsHEADER.h"
#include <errno.h>


unsigned yaffs_trace_mask =

	YAFFS_TRACE_SCAN |
	YAFFS_TRACE_GC |
	YAFFS_TRACE_ERASE |
	YAFFS_TRACE_ERROR |
	YAFFS_TRACE_TRACING |
	YAFFS_TRACE_ALLOCATE |
	YAFFS_TRACE_BAD_BLOCKS |
	YAFFS_TRACE_VERIFY |
	0;



/* Configure the devices that will be used */

#include "yaffs_flashif2.h"
//#include "yaffs_m18_drv.h"
//#include "yaffs_nor_drv.h"
#include "yaffs_nand_drv.h"

int yaffs_start_up(void)
{
	struct yaffs_dev *dev;
	struct nand_chip *chp;
	// NAND Flash parameters 
	nanddrv_initialise();
	dev = (struct yaffs_dev*)sdcalloc(1,sizeof (struct yaffs_dev)); configASSERT(dev)
	chp = (struct nand_chip*)sdcalloc(1,sizeof (struct nand_chip)); configASSERT(chp)

	dev->param.name = "nand";				
	dev->driver_context = NULL;
	dev->param.start_block = 1;
	dev->param.end_block = NANDFLASH_BLOCK_DATALIMIT; //TUFIXDONE: Check this value, to see if it is the same as the 500 blocks limit - it's the same, but 500 = number of dirty blocks while 900 is the total number of blocks
	dev->param.chunks_per_block = 64;
	dev->param.total_bytes_per_chunk = 2048;
	dev->param.spare_bytes_per_chunk = 64;
	dev->param.is_yaffs2 = 1;
	dev->param.use_nand_ecc = 1;
	dev->param.n_reserved_blocks = 5;
	dev->param.wide_tnodes_disabled=0;
	dev->param.refresh_period = 1000;
	dev->param.enable_xattr = 1;	
	dev->param.n_caches = 10;
	dev->tagger.write_chunk_tags_fn = NULL;
	dev->tagger.read_chunk_tags_fn = NULL;
	dev->tagger.query_block_fn = NULL;
	dev->tagger.mark_bad_fn = NULL;
	yaffs_tags_compat_install(dev);
	yaffs_tags_marshall_install(dev);
	
	chp->blocks = NANDFLASH_BLOCK_DATALIMIT;
	chp->pages_per_block = 64;
	chp->data_bytes_per_page= 2048;
	chp->spare_bytes_per_page = 64;
	
	chp->bus_width_shift = 0;
	yaffs_nand_install_drv(dev,chp);
	
	// The yaffs device has been configured, install it into yaffs 
	yaffs_add_device(dev);

	return YAFFS_OK;
}


void Test_Yaffs2(void) // Serve as an example for yaffs2
{
	int f,i;
	int *str,*str1;
	struct yaffs_stat *buf;
	struct yaffs_dev *dev;	
	struct yaffs_obj *dir;
	struct list_head *j;
	YCHAR buffer[YAFFS_MAX_NAME_LENGTH + 1];
	struct yaffs_obj *l;
	
	YCHAR *restOfPath = (YCHAR*)sdmalloc(YAFFS_MAX_NAME_LENGTH + 1); // need this malloc here or str[i] = *restOfPath; causes trouble
	dev = (struct yaffs_dev*)sdmalloc(sizeof (struct yaffs_dev));
	buf = (struct yaffs_stat*)sdmalloc(sizeof (struct yaffs_stat));
	dev = (struct yaffs_dev*) yaffsfs_FindDevice("/nand/file50.txt", &restOfPath);
	configASSERT(restOfPath && dev && buf && dev);

	dir = dev->root_dir;
	lpc_printf("LIST:\n\r");
	list_for_each(j, &dir->variant.dir_variant.children) {
		l = list_entry(j, struct yaffs_obj, siblings);
		yaffs_get_obj_name(l, buffer, YAFFS_MAX_NAME_LENGTH + 1);
		lpc_printf("----: %s \n\r", buffer);
	}
	
/*	access flag:
	 O_CREAT: Create file if it does not already exist.
	 O_EXCL Only use with O_CREAT. Create file if it does not exist. If the filealready exists then fail.
	 O_TRUNC If the file exists, and opening with write access, then truncate the file to zero bytes long.
	 O_APPEND Regardless of the handle position, always write to the end of the file.
	 O_RDWR Open for read/write access.
	 O_WRONLY Open for write access and no read access.
	 0 (zero) If neither O_RDWR or O_WRONLY is set then the file will be opened for read-only access.
	creation mode:
	 S_IREAD The file may be opened for read access.
	 S_IWRITE The file may be opened for write access.
   S_IEXEC The file may be opened for execution. This is not enforced by yaffs.
	 Return:  Fail: -1, !-1 (handle) otherwise
	 Example:
	Where they are not mutually exclusive, these flags are typically combined. For example:
	O_CREAT | O_TRUNC | O_RDWR : Create a new file if it does not exist otherwise if the file
	already exists then truncate to zero length. Typically used to overwrite a file or create if it
	does not already exist.
	O_CREAT | OEXCL | O_WRONLY : Create a new file, opening it for write only. Fails if file
	already exists. */
	
//	yaffs_open(path, O_RDWR | O_CREAT | O_TRUNC, S_IREAD | S_IWRITE);

	
	//yaffs_mkdir("/nand/dir1", S_IREAD| S_IWRITE);
	
	f = yaffs_open("/nand/file50.txt", O_RDONLY, S_IREAD);

	if (f != -1)
	{	
					lpc_printf("f=%d\n\r",f);
					//memset (str1, 0x00, 35);
					yaffs_fstat(f, buf) ;	
					lpc_printf("buf = %llu", buf->st_size);
					str1 = (int*) sdcalloc(1,buf->st_size);
					configASSERT(str1);
					lpc_printf("reading\n\r");
					yaffs_read(f, str1, (unsigned int) buf->st_size);
					yaffs_close(f); 
					lpc_printf("closed\n\r");
			for (i=0;i<1000000;i++)
			{	
				if ((i%1000) == 0)
				{
			//		lpc_printf("%d\n\r", str1[i]);
				}
			}
	}
	else
	{ 
			lpc_printf("----> There is no file. Let's create it...\r\n");

			f = yaffs_open("/nand/file50.txt", O_CREAT | O_TRUNC | O_RDWR, (S_IREAD | S_IWRITE));
			lpc_printf("fcreate = %d\n\r",f);
			if ( f != -1)
			{
					str = (int*) sdcalloc(1,1000000*4);
					configASSERT(str);
					for (i=0;i<1000000;i++)
					{	
						str[i] = i;
						if ((i%1000) == 0)
						{
							lpc_printf("%d\n\r", str[i]);
						}
					}
					yaffs_write(f, str, 4000000);
					yaffs_flush(f);
					yaffs_close(f);
					lpc_printf("----> Done! Restart the app to read the file.\r\n");
			}
	}
}


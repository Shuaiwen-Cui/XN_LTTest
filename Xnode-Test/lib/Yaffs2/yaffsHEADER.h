#include "nand_chip.h"
#include "nand_store.h"
#include "nanddrv.h"
#include "unistd.h"
#include "yaffs_allocator.h"
#include "yaffs_attribs.h"
#include "yaffs_bitmap.h"
#include "yaffs_checkptrw.h"
#include "yaffs_ecc.h"
#include "yaffs_endian.h"
#include "yaffs_fileem2k.h"
#include "yaffs_flashif.h"
#include "yaffs_flashif2.h"
#include "yaffs_getblockinfo.h"
#include "yaffs_guts.h"
#include "yaffs_hweight.h"
#include "yaffs_list.h"
#include "yaffs_m18_drv.h"
#include "yaffs_nameval.h"
#include "yaffs_nand.h"
#include "yaffs_nand_drv.h"
#include "yaffs_nandemul2k.h"
#include "yaffs_nandsim_file.h"
#include "yaffs_osglue.h"
#include "yaffs_packedtags1.h"
#include "yaffs_packedtags2.h"
#include "yaffs_stat.h"
#include "yaffs_summary.h"
#include "yaffs_tagscompat.h"
#include "yaffs_tagsmarshall.h"
#include "yaffs_trace.h"
#include "yaffs_yaffs1.h"
#include "yaffs_yaffs2.h"
#include "yaffscfg2k.h"
#include "yaffsfs.h"
#include "ydirectenv.h"
#include "yportenv_multi.h"
#include "nandflash_k9f1g08u0a.h"

void yaffs_verify_blocks(struct yaffs_dev *dev);
void yaffs_verify_free_chunks(struct yaffs_dev *dev);
void yaffs_verify_objects(struct yaffs_dev *dev);
int nanddrv_initialise(void);
void yaffs_verify_oh(struct yaffs_obj *obj, struct yaffs_obj_hdr *oh,struct yaffs_ext_tags *tags, int parent_check);
void yaffs_verify_obj_in_dir(struct yaffs_obj *obj);
int yaffs_verify_file_sane(struct yaffs_obj *in);
void yaffs_verify_dir(struct yaffs_obj *directory);
int yaffs_skip_verification(struct yaffs_dev *dev);
void yaffs_verify_blk(struct yaffs_dev *dev, struct yaffs_block_info *bi, int n);
void yaffs_verify_collected_blk(struct yaffs_dev *dev,struct yaffs_block_info *bi, int n);
void yaffs_qsort(void *aa, size_t n, size_t es, int (*cmp)(const void *, const void *));
struct yaffs_dev *yaffsfs_FindDevice(const YCHAR *path, YCHAR **restOfPath);

#include <xnode.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <NodeReset.h>
#include "sdmalloc.h"
#include "debug_frmwrk.h"

#define SDRAM_BASE_ADDRESS (0x28000000)
#define SDRAM_SIZE         (0x2000000)
#define SDRAM_END_ADDRESS  (SDRAM_BASE_ADDRESS + SDRAM_SIZE)
#define USED               (1)

typedef struct {
	unsigned int prebuf;
	unsigned int size;
	unsigned int postbuf;
} block_t;

static struct {
	block_t* free;
	block_t* heap;
} sdram;

#define MAGIC (0xDEADBEEF)
#define debug_set(p) do { p->prebuf = p->postbuf = MAGIC; } while (0)
#define debug_check(p) do { \
	if (p->prebuf != MAGIC || p->postbuf != MAGIC) { \
		lpc_printf("-- ERROR: sdram heap corruption detected\r\n"); \
		die(); \
	} } while (0)

static block_t* compact(block_t *p, unsigned int nsize);
static uint32_t total_size = 0;
static bool sddebug = false, sdlocked = false;

void sddebugenable(void)
{
		sddebug = true;
}

void sddebugdisable(void)
{
		sddebug = false;
}

unsigned int sdallocated(void)
{
	return total_size;
}

unsigned int sdavailable(void)
{
	return (SDRAM_SIZE - total_size);
}

void sdlock(void)
{
	bool res = false;
	do {
		//uint32_t int_status	= __disable_irq();
		if (!sdlocked) {
			res = sdlocked = true;
		} 
		//if (!int_status) {
		//	__enable_irq();
		//}
	} while (!res);
}

void sdunlock(void)
{
	//uint32_t int_status	= __disable_irq();
  sdlocked = false;
	//if (!int_status) {
	//	__enable_irq();
	//}
}

void SDRAM_Init(void)
{
	block_t *p;
	sdram.free = sdram.heap = (block_t *)SDRAM_BASE_ADDRESS;
	sdram.free->size = sdram.heap->size = SDRAM_SIZE - sizeof (block_t);
	debug_set(sdram.free);
	p = (block_t *)((char *)SDRAM_BASE_ADDRESS + SDRAM_SIZE - sizeof (block_t));
	p->size = 0;
	debug_set(p);
}

void *sdmalloc(unsigned int size)
{
	unsigned int fsize;
	block_t *p;

	if (size == 0) {
		return 0;
	}

	size += 3 + sizeof (block_t);
	size >>= 2;
	size <<= 2;
	
	sdlock();
	
	if (sdram.free == 0 || size > sdram.free->size) {
		sdram.free = compact(sdram.heap, size);
		if (sdram.free == 0) {
			sdunlock();
			return 0;
		}
	}
	debug_check(sdram.free);

	p = sdram.free;
	fsize = sdram.free->size;

	if (fsize >= size + sizeof (block_t)) {
		sdram.free = (block_t *)((unsigned int)p + size);
		sdram.free->size = fsize - size;
	} else {
		sdram.free = 0;
		size = fsize;
	}
	debug_set(sdram.free);
	
	total_size += size;
	p->size = size | USED;
	debug_set(p);
	sdunlock();
	
	if (sddebug) {
	  lpc_printf("-- sdmalloc %08X\t+%10u = %u\r\n", (unsigned int)p + sizeof (block_t), size, total_size);
  }

	return (void *)((unsigned int)p + sizeof (block_t));
}

void *sdcalloc(unsigned int nmemb, unsigned int size)
{
	char *p = (char *)sdmalloc(nmemb * size);
	if (!p) {
		return (void *)p;
	}
	memset(p, 0, nmemb * size);
	return (void *)p;
}

void sdfree(void *ptr)
{
	if (ptr) {
		block_t *p;
		sdlock();
		p = (block_t *)((unsigned int)ptr - sizeof (block_t));
		debug_check(p);
		p->size &= ~USED;
		total_size -= p->size;
		sdunlock();

		if (sddebug) {
			lpc_printf("-- sdfree   %08X\t-%10u = %u\r\n", (unsigned int)ptr, p->size, total_size);
		}
	}
}

void sdcompact(void)
{
	sdram.free = compact(sdram.heap, SDRAM_SIZE);
}

static block_t* compact(block_t *p, unsigned int nsize)
{
	unsigned int bsize, psize;
	block_t *best;

	debug_check(p);

	best = p;
	bsize = 0;

	while (psize = p->size, psize) {
		if (psize & USED) {
			if (bsize != 0) {
				best->size = bsize;
				if (bsize >= nsize) {
					return best;
				}
			}
			bsize = 0;
			best = p = (block_t *)((unsigned int)p + (psize & ~USED));
		} else {
			bsize += psize;
			p = (block_t *)((unsigned int)p + psize);
		}
		debug_check(p);
	}

	if (bsize != 0) {
		best->size = bsize;
		if (bsize >= nsize) {
			return best;
		}
	}

	return 0;
}

#undef debug_set
#undef debug_unset
#undef MAGIC
#undef USED

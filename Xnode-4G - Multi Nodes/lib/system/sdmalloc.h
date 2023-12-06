#ifndef SDMALLOC_H
#define SDMALLOC_H

void SDRAM_Init(void);
void *sdmalloc(unsigned int size);
void *sdcalloc(unsigned int nmemb, unsigned int size);
void sdfree(void *ptr);
void sdcompact(void); 
unsigned int sdallocated(void);
unsigned int sdavailable(void);
void sddebugenable(void);
void sddebugdisable(void);

#endif /* SDMALLOC_H */

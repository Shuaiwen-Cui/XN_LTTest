#ifndef _FILTER_H_
#define _FILTER_H_
#include "filter_2.h" // sum of coef's = 1
#include "filter_3.h" // sum of coef's = 1
#include "filter_5.h" // sum of coef's = 1
#include "filter_10.h" // sum of coef's = 1

int lookup(unsigned int rate);
void filter(float *input, float *output, unsigned int number_input, unsigned int rate);
void filtertest_SETUP(unsigned int blockSize, unsigned int numSamples, unsigned int rate, unsigned int channel); // this number_input is the total expected length of output, it must be cut off a litle bit
void filtertest(float *input, float *output, unsigned int blockSize, unsigned int rate, unsigned int sample, unsigned int channel); // this number_input is the total expected length of output, it must be cut off a litle bit


#endif /* _FILTER_H_ */

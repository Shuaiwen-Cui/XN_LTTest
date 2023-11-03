#ifndef _FILTER_H_
#define _FILTER_H_

#define order_2 88
#define order_3 132
#define order_5 220
#define order_10 439

void filter(float *input, float *output, unsigned int number_input, unsigned int rate);

#endif /* _FILTER_H_ */

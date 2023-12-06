#ifndef GATEWAY
#include <xnode.h>
#include <string.h>
#include <sdmalloc.h>
#include "arm_math.h"
#include <inttypes.h>
#include "Sensing.h"
#include "filter.h"

// Initialize filter variables once
arm_fir_decimate_instance_f32 S_1[MAX_SENSOR_CHANNELS], S_2[MAX_SENSOR_CHANNELS];
float32_t *firStateF32_1[MAX_SENSOR_CHANNELS] = { NULL }, *firStateF32_2[MAX_SENSOR_CHANNELS] = { NULL };
unsigned int idx_1=0, idx_2=0;
bool filtertwice = false;	// for cases like downsampling by factor of 20,50,100, this is true (10 ->2, 10->5, 10->10)
float32_t *output_temp[MAX_SENSOR_CHANNELS];
int ignore = 0;	// how many data points to be ignored (and then overwritten)
int scale = 0, temp = 0;	// temp is a temporary variable take values of 0 or scale to rewrite useful output

typedef struct {
uint8_t order; 	// rate
uint32_t size;	// number of taps
float coef[441];  // make it bigger than the biggest filter - but not too big - if more filters involve this can be made a manually set variable outside of typedef struct
} filter_info_t;

static const filter_info_t filters[4] = { // again, when more filters are added, 4 should be increased to the total number of filters
{order_2, size_2, coef_2},
{order_3, size_3, coef_3},
{order_5, size_5, coef_5},
{order_10, size_10, coef_10}
};
  
int lookup(unsigned int rate){ // given rate, look up the index in filters[] to be used
	unsigned int i, filters_count;
	filters_count = sizeof(filters)/sizeof(filters[0]);
	for (i = filters_count-1;; i--) {	// Start from high to lower order
		if ((rate%filters[i].order)==0){
			return i;
		} else if (i == 0) {
			break;
		}
	}
	lpc_printf("ERROR: invalid decimation rate.\r\n");
	return 0;
}

void filtertest_SETUP(unsigned int blockSize, unsigned int numSamples, unsigned int rate, unsigned int channel) // this number_input is the total expected length of output, it must be cut off a litle bit
{
	idx_1 = lookup(rate);		// look up rate 
	if (filters[idx_1].order != rate) {
		filtertwice = true; // check if need another filter - right now only support filtering twice (rate = 2,5,10,20,50,100)
		idx_2 = lookup((int)(rate/filters[idx_1].order));
	}
	firStateF32_1[channel]=(float32_t*)sdcalloc(blockSize + filters[idx_1].size - 1, sizeof(float32_t));	// set up first filter
	arm_fir_decimate_init_f32(&S_1[channel], filters[idx_1].size ,filters[idx_1].order, (float32_t *)&filters[idx_1].coef[0], & firStateF32_1[channel][0], blockSize);// set up first filter
	ignore =  filters[idx_1].size; // size of output data that is not used
	if (filtertwice){ // another filter needs to be setup if 2 filters are used
		output_temp[channel] = (float32_t*)sdcalloc( numSamples/filters[idx_1].order, sizeof(float32_t));// set up temporary output	
		firStateF32_2[channel]=(float32_t*)sdcalloc((int) blockSize/filters[idx_1].order + filters[idx_2].size - 1, sizeof(float32_t));// set up second filter
		arm_fir_decimate_init_f32(&S_2[channel], filters[idx_2].size ,filters[idx_2].order, (float32_t *)&filters[idx_2].coef[0], & firStateF32_2[channel][0], blockSize/filters[idx_1].order);// set up second filter
		ignore = filters[idx_2].size*filters[idx_1].order + filters[idx_1].size; // size of input data that is not used
	}
	scale = (int) ignore/blockSize+1; // number of blocks to be ignored when starting to get meaningful data --> the delay will be ((blockSize*scale - num_taps)/2)
}

void filtertest(float *input, float *output, unsigned int blockSize, unsigned int rate, unsigned int sample, unsigned int channel) // this number_input is the total expected length of output, it must be cut off a litle bit
{
	if (sample<(ignore+blockSize)){
		temp = 0;	// temp = 0 so result will be ignored
	} else {
		temp = scale;  // start writing ouput again from 0, overwriting the not-useful data points
	}
		if (!filtertwice){
			arm_fir_decimate_f32(&S_1[channel], &input[sample-blockSize], &output[(sample-(1+temp)*blockSize)/filters[idx_1].order], blockSize);  

		} else	{
			arm_fir_decimate_f32(&S_1[channel], &input[sample-10], &output_temp[channel][(sample-10)/10], 10);   // output is a temporary block
			if ((sample%blockSize) == 0){		
				arm_fir_decimate_f32(&S_2[channel], &output_temp[channel][(sample-blockSize)/10], &output[(sample-blockSize)/rate-temp], blockSize/10);
			}
		}
}
#endif

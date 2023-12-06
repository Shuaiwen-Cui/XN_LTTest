#if defined(GW_SENSE) || !defined(GATEWAY)
#include <xnode.h>
#include <string.h>
#include <sdmalloc.h>
#include "arm_math.h"
#include <inttypes.h>
#include "Sensing.h"
#include "filter.h"

// Initialize filter variables once
arm_fir_decimate_instance_f32 S_1[MAX_SENSOR_CHANNELS], S_2[MAX_SENSOR_CHANNELS];
arm_fir_instance_f32 S_disp[MAX_SENSOR_CHANNELS];
float32_t *firStateF32_1[MAX_SENSOR_CHANNELS] = { NULL }, *firStateF32_2[MAX_SENSOR_CHANNELS] = { NULL }, *firStateF32[MAX_SENSOR_CHANNELS] = { NULL };
unsigned int idx_1=0, idx_2=0;
bool filtertwice = false;	// for cases like downsampling by factor of 20,50,100, this is true (10 ->2, 10->5, 10->10)
float32_t *output_temp[MAX_SENSOR_CHANNELS];
int ignore = 0;	// how many data points to be ignored (and then overwritten)
int scale = 0, temp = 0;	// temp is a temporary variable take values of 0 or scale to rewrite useful output
float ignored_data;

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
	configASSERT(firStateF32_1[channel]);
	arm_fir_decimate_init_f32(&S_1[channel], filters[idx_1].size ,filters[idx_1].order, (float32_t *)&filters[idx_1].coef[0], & firStateF32_1[channel][0], blockSize);// set up first filter
	ignore =  filters[idx_1].size; // size of output data that is not used
	if (filtertwice){ // another filter needs to be setup if 2 filters are used
		output_temp[channel] = (float32_t*)sdcalloc( numSamples/filters[idx_1].order, sizeof(float32_t));// set up temporary output	
		firStateF32_2[channel]=(float32_t*)sdcalloc((int) blockSize/filters[idx_1].order + filters[idx_2].size - 1, sizeof(float32_t));// set up second filter
		configASSERT(output_temp[channel] && firStateF32_2[channel]);		
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

// This part is for disp est
#define length_disp_1Hz 523
float coef_disp[length_disp_1Hz] = {7.69145273512071e-06,7.86710890153911e-06,8.03763216094996e-06,8.20276337722586e-06,8.36205201983769e-06,8.51523539560004e-06,8.6618572356477e-06,8.80165289587827e-06,8.93416195853414e-06,9.05911954137997e-06,9.17606277034402e-06,9.28472834177389e-06,9.3846527178572e-06,9.47557609077841e-06,9.55703614771677e-06,9.62877859038936e-06,9.69034431130438e-06,9.74148662703151e-06,9.7817517020631e-06,9.81090265921141e-06,9.82849308188996e-06,9.8342981703106e-06,9.82788114723142e-06,9.80903163412601e-06,9.77732477944333e-06,9.73256703465964e-06,9.67434781784831e-06,9.60249287551417e-06,9.51660828776623e-06,9.41654160808938e-06,9.30191800963718e-06,9.17260940163118e-06,9.0282625092322e-06,8.86877617208851e-06,8.69382114288788e-06,8.5033257807197e-06,8.29698734574143e-06,8.07476630749367e-06,7.83638890511848e-06,7.58185029858834e-06,7.31090815557333e-06,7.02359488173574e-06,6.71970198663937e-06,6.39930163784186e-06,6.06222154915436e-06,5.70857611225535e-06,5.33823154112183e-06,4.95134684431557e-06,4.5478289494959e-06,4.12788378941706e-06,3.69146112007447e-06,3.23881600382632e-06,2.76994302389534e-06,2.28514845891806e-06,1.78447358519462e-06,1.2682778484056e-06,7.36650933146074e-07,1.90007249563623e-07,-3.71513562701797e-07,-9.47440502577876e-07,-1.53758261509922e-06,-2.14141087050925e-06,-2.7586818427948e-06,-3.38880724221129e-06,-4.03149023862518e-06,-4.68608227515068e-06,-5.35223242511452e-06,-6.02923109902564e-06,-6.71667283972723e-06,-7.41378651863225e-06,-8.12011199017481e-06,-8.83481635608364e-06,-9.55738491763854e-06,-1.0286923068666e-05,-1.10228620024025e-05,-1.17642457748227e-05,-1.25104522421677e-05,-1.32604648160965e-05,-1.40136091282171e-05,-1.47688089773144e-05,-1.55253392385977e-05,-1.6282065480833e-05,-1.70382136605631e-05,-1.77925928632817e-05,-1.85443823466708e-05,-1.92923368349161e-05,-2.00355915001293e-05,-2.0772849212425e-05,-2.15032040752486e-05,-2.22253100058188e-05,-2.2938223468162e-05,-2.36405527288664e-05,-2.43313204613473e-05,-2.50090929904432e-05,-2.56728634729154e-05,-2.6321160411126e-05,-2.6952952148143e-05,-2.75667338954331e-05,-2.81614543163446e-05,-2.8735580275282e-05,-2.92880463209211e-05,-2.98172963245557e-05,-3.03222567143272e-05,-3.08013541281704e-05,-3.12535132928005e-05,-3.16771497717499e-05,-3.2071193428043e-05,-3.24340553000054e-05,-3.27646776346876e-05,-3.30614738732242e-05,-3.33234062933752e-05,-3.35488980319712e-05,-3.37369394296842e-05,-3.38859709602518e-05,-3.39950194290515e-05,-3.40625506170619e-05,-3.40876365472989e-05,-3.4068776585533e-05,-3.4005097055493e-05,-3.38951394678622e-05,-3.37380938367365e-05,-3.35325526329873e-05,-3.32777792311862e-05,-3.29724261026273e-05,-3.26158399042452e-05,-3.22067423399717e-05,-3.17445734915748e-05,-3.12281336840703e-05,-3.06569667534438e-05,-3.00299611519779e-05,-2.93467749500923e-05,-2.86063943100587e-05,-2.78086021293681e-05,-2.69524918956935e-05,-2.60379819980155e-05,-2.50642828510817e-05,-2.40314590288114e-05,-2.29388474120349e-05,-2.17866694373724e-05,-2.05743978767567e-05,-1.93024216450585e-05,-1.79703586627396e-05,-1.65787758281102e-05,-1.51274452442332e-05,-1.36171221381241e-05,-1.20477415483928e-05,-1.04202571653546e-05,-8.7347753753587e-06,-6.99245820426513e-06,-5.19359139628834e-06,-3.33955487040188e-06,-1.43082127392358e-06,5.3100239083552e-07,2.54524955723929e-06,4.61007736943793e-06,6.7246189195395e-06,8.88678720030051e-06,1.10955092010797e-05,1.33484459893092e-05,1.56443134174211e-05,1.79805134279266e-05,2.03555464968387e-05,2.27665488652692e-05,2.52118022556667e-05,2.76881713010525e-05,3.01937165232691e-05,3.27250257543684e-05,3.52799368663183e-05,3.78547564519439e-05,4.0447099349263e-05,4.30529872652156e-05,4.56698127520208e-05,4.82933098081557e-05,5.09206506469669e-05,5.35472795872289e-05,5.61701517151334e-05,5.87844205710934e-05,6.13868286561473e-05,6.39722385206295e-05,6.65371860177738e-05,6.90762433895759e-05,7.15857472389823e-05,7.40599810724491e-05,7.64950911652182e-05,7.88850747395459e-05,8.1225898256061e-05,8.35112759586987e-05,8.57370066637286e-05,8.78965257601277e-05,8.99854783160014e-05,9.1997025754289e-05,9.39266750891901e-05,9.57673193631505e-05,9.75143451058592e-05,9.91603831729609e-05,0.000100700719138293,0.000102127728361468,0.000103436617042304,0.000104619512094861,0.000105671564087166,0.000106584658729664,0.00010735391698635,0.000107970990592648,0.000108430999370746,0.000108725368047759,0.000108849246381266,0.000108793838493484,0.000108554357991674,0.000108121793867187,0.00010749146060533,0.000106654136165171,0.00010560527640186,0.000104335450418974,0.000102840299841828,0.000101110184499933,9.91409806699116e-05,9.69228350559835e-05,9.44519126903421e-05,9.17181388172434e-05,8.87180275230734e-05,8.54412684406746e-05,8.18847924860123e-05,7.80380319994068e-05,7.38984116884822e-05,6.945507515939e-05,6.47060293649293e-05,5.96400850250736e-05,5.42559344270765e-05,4.85419945765201e-05,4.24977651691822e-05,3.61111865618127e-05,2.9382713027747e-05,2.22996956485037e-05,1.48637242788687e-05,7.06140757109785e-06,-1.10430143752552e-06,-9.64774607262387e-06,-1.85643168296966e-05,-2.78695988485447e-05,-3.75569378399872e-05,-4.76435680258975e-05,-5.81202603586365e-05,-6.9006482749618e-05,-8.02896827878486e-05,-9.19924249076515e-05,-0.000104097733013188,-0.000116632588008723,-0.00012957389468758,-0.000142955165294624,-0.000156744417705836,-0.000170985295598448,-0.000185632066564376,-0.000200745153612661,-0.000216255646207265,-0.00023225450714931,-0.000248628623042163,-0.000265533316131903,-0.000282752822159189,-0.000300618183981633,-0.000318564583421093,-0.000337811906432565,-0.00035252085180293,-0.000337811906432565,-0.000318564583421093,-0.000300618183981633,-0.000282752822159189,-0.000265533316131903,-0.000248628623042163,-0.00023225450714931,-0.000216255646207265,-0.000200745153612661,-0.000185632066564376,-0.000170985295598448,-0.000156744417705836,-0.000142955165294624,-0.00012957389468758,-0.000116632588008723,-0.000104097733013188,-9.19924249076515e-05,-8.02896827878486e-05,-6.9006482749618e-05,-5.81202603586365e-05,-4.76435680258975e-05,-3.75569378399872e-05,-2.78695988485447e-05,-1.85643168296966e-05,-9.64774607262387e-06,-1.10430143752552e-06,7.06140757109785e-06,1.48637242788687e-05,2.22996956485037e-05,2.9382713027747e-05,3.61111865618127e-05,4.24977651691822e-05,4.85419945765201e-05,5.42559344270765e-05,5.96400850250736e-05,6.47060293649293e-05,6.945507515939e-05,7.38984116884822e-05,7.80380319994068e-05,8.18847924860123e-05,8.54412684406746e-05,8.87180275230734e-05,9.17181388172434e-05,9.44519126903421e-05,9.69228350559835e-05,9.91409806699116e-05,0.000101110184499933,0.000102840299841828,0.000104335450418974,0.00010560527640186,0.000106654136165171,0.00010749146060533,0.000108121793867187,0.000108554357991674,0.000108793838493484,0.000108849246381266,0.000108725368047759,0.000108430999370746,0.000107970990592648,0.00010735391698635,0.000106584658729664,0.000105671564087166,0.000104619512094861,0.000103436617042304,0.000102127728361468,0.000100700719138293,9.91603831729609e-05,9.75143451058592e-05,9.57673193631505e-05,9.39266750891901e-05,9.1997025754289e-05,8.99854783160014e-05,8.78965257601277e-05,8.57370066637286e-05,8.35112759586987e-05,8.1225898256061e-05,7.88850747395459e-05,7.64950911652182e-05,7.40599810724491e-05,7.15857472389823e-05,6.90762433895759e-05,6.65371860177738e-05,6.39722385206295e-05,6.13868286561473e-05,5.87844205710934e-05,5.61701517151334e-05,5.35472795872289e-05,5.09206506469669e-05,4.82933098081557e-05,4.56698127520208e-05,4.30529872652156e-05,4.0447099349263e-05,3.78547564519439e-05,3.52799368663183e-05,3.27250257543684e-05,3.01937165232691e-05,2.76881713010525e-05,2.52118022556667e-05,2.27665488652692e-05,2.03555464968387e-05,1.79805134279266e-05,1.56443134174211e-05,1.33484459893092e-05,1.10955092010797e-05,8.88678720030051e-06,6.7246189195395e-06,4.61007736943793e-06,2.54524955723929e-06,5.3100239083552e-07,-1.43082127392358e-06,-3.33955487040188e-06,-5.19359139628834e-06,-6.99245820426513e-06,-8.7347753753587e-06,-1.04202571653546e-05,-1.20477415483928e-05,-1.36171221381241e-05,-1.51274452442332e-05,-1.65787758281102e-05,-1.79703586627396e-05,-1.93024216450585e-05,-2.05743978767567e-05,-2.17866694373724e-05,-2.29388474120349e-05,-2.40314590288114e-05,-2.50642828510817e-05,-2.60379819980155e-05,-2.69524918956935e-05,-2.78086021293681e-05,-2.86063943100587e-05,-2.93467749500923e-05,-3.00299611519779e-05,-3.06569667534438e-05,-3.12281336840703e-05,-3.17445734915748e-05,-3.22067423399717e-05,-3.26158399042452e-05,-3.29724261026273e-05,-3.32777792311862e-05,-3.35325526329873e-05,-3.37380938367365e-05,-3.38951394678622e-05,-3.4005097055493e-05,-3.4068776585533e-05,-3.40876365472989e-05,-3.40625506170619e-05,-3.39950194290515e-05,-3.38859709602518e-05,-3.37369394296842e-05,-3.35488980319712e-05,-3.33234062933752e-05,-3.30614738732242e-05,-3.27646776346876e-05,-3.24340553000054e-05,-3.2071193428043e-05,-3.16771497717499e-05,-3.12535132928005e-05,-3.08013541281704e-05,-3.03222567143272e-05,-2.98172963245557e-05,-2.92880463209211e-05,-2.8735580275282e-05,-2.81614543163446e-05,-2.75667338954331e-05,-2.6952952148143e-05,-2.6321160411126e-05,-2.56728634729154e-05,-2.50090929904432e-05,-2.43313204613473e-05,-2.36405527288664e-05,-2.2938223468162e-05,-2.22253100058188e-05,-2.15032040752486e-05,-2.0772849212425e-05,-2.00355915001293e-05,-1.92923368349161e-05,-1.85443823466708e-05,-1.77925928632817e-05,-1.70382136605631e-05,-1.6282065480833e-05,-1.55253392385977e-05,-1.47688089773144e-05,-1.40136091282171e-05,-1.32604648160965e-05,-1.25104522421677e-05,-1.17642457748227e-05,-1.10228620024025e-05,-1.0286923068666e-05,-9.55738491763854e-06,-8.83481635608364e-06,-8.12011199017481e-06,-7.41378651863225e-06,-6.71667283972723e-06,-6.02923109902564e-06,-5.35223242511452e-06,-4.68608227515068e-06,-4.03149023862518e-06,-3.38880724221129e-06,-2.7586818427948e-06,-2.14141087050925e-06,-1.53758261509922e-06,-9.47440502577876e-07,-3.71513562701797e-07,1.90007249563623e-07,7.36650933146074e-07,1.2682778484056e-06,1.78447358519462e-06,2.28514845891806e-06,2.76994302389534e-06,3.23881600382632e-06,3.69146112007447e-06,4.12788378941706e-06,4.5478289494959e-06,4.95134684431557e-06,5.33823154112183e-06,5.70857611225535e-06,6.06222154915436e-06,6.39930163784186e-06,6.71970198663937e-06,7.02359488173574e-06,7.31090815557333e-06,7.58185029858834e-06,7.83638890511848e-06,8.07476630749367e-06,8.29698734574143e-06,8.5033257807197e-06,8.69382114288788e-06,8.86877617208851e-06,9.0282625092322e-06,9.17260940163118e-06,9.30191800963718e-06,9.41654160808938e-06,9.51660828776623e-06,9.60249287551417e-06,9.67434781784831e-06,9.73256703465964e-06,9.77732477944333e-06,9.80903163412601e-06,9.82788114723142e-06,9.8342981703106e-06,9.82849308188996e-06,9.81090265921141e-06,9.7817517020631e-06,9.74148662703151e-06,9.69034431130438e-06,9.62877859038936e-06,9.55703614771677e-06,9.47557609077841e-06,9.3846527178572e-06,9.28472834177389e-06,9.17606277034402e-06,9.05911954137997e-06,8.93416195853414e-06,8.80165289587827e-06,8.6618572356477e-06,8.51523539560004e-06,8.36205201983769e-06,8.20276337722586e-06,8.03763216094996e-06,7.86710890153911e-06,7.69145273512071e-06}; // if define this in a separate .h file and read from it (as define), then reading is not correct after the element #420, not sure why

void filter_disp_SETUP(unsigned int blockSize, unsigned int channel) 
{
	firStateF32[channel]=(float32_t*)sdcalloc(blockSize + length_disp_1Hz - 1, sizeof(float32_t));	// set up filter
	configASSERT(firStateF32[channel]);		
	arm_fir_init_f32(&S_disp[channel], length_disp_1Hz,(float32_t *) &coef_disp[0],&firStateF32[channel][0],blockSize );
}

void filter_disp(float *input, float *output, unsigned int blockSize, unsigned int channel,unsigned int sample) 
{
	if (sample<length_disp_1Hz+blockSize) // for ignored part, still must process to have information of the initial part
	{
		arm_fir_f32(&S_disp[channel], &input[sample], &ignored_data, blockSize);
	}
	else
	{
		arm_fir_f32(&S_disp[channel], &input[sample], &output[sample-(length_disp_1Hz+blockSize)], blockSize);
	}
} 
#endif
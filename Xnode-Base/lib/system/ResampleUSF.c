/* ************************************************************************
 * 
 * Unless otherwise noted, the files which comprise the Structural Health
 * Monitoring software toolsuite (SHM) are subject to the following
 * restrictions.
 * 
 * The SHM software is NOT in the public domain.  However, it is freely
 * available without fee for education, research, and non-profit purposes.
 * By obtaining copies of this and other files that comprise the SHM
 * software, you, the Licensee, agree to abide by the following conditions
 * and understandings with respect to the copyrighted software:
 * 
 * 1.  The software is copyrighted in the name of the Board of Trustees
 *     of the University of Illinois (UI), and ownership of the software
 *     remains with the UI.
 * 
 * 2.  Permission to use, copy, and modify this software and its
 *     documentation for education, research, and non-profit purposes is
 *     hereby granted to the Licensee, provided that the copyright
 *     notice, the original author's names and unit identification, and
 *     this permission notice appear on all such copies, and that no
 *     charge be made for such copies.  Any entity desiring permission to
 *     incorporate this software into commercial products should contact:
 * 
 *          Professor Gul A. Agha                 agha@cs.uiuc.edu
 *          University of Illinois at Urbana-Champaign
 *          Department of Computer Science
 *          2104 Siebel Center
 *          201 North Goodwin Avenue
 *          Urbana, Illinois  61801
 *          USA
 * 
 * 3.  Licensee may not use the name, logo, or any other symbol of the UI
 *     nor the names of any of its employees nor any adaptation thereof
 *     in advertising or publicity pertaining to the software without
 *     specific prior written approval of the UI.
 * 
 * 4.  THE UI MAKES NO REPRESENTATIONS ABOUT THE SUITABILITY OF THE
 *     SOFTWARE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS
 *     OR IMPLIED WARRANTY.
 * 
 * 5.  The UI shall not be liable for any damages suffered by Licensee
 *     from the use of this software.
 * 
 * 6.  The software was developed under agreements between the UI and the
 *     Federal Government which entitles the Government to certain
 *     rights.
 * 
 * ************************************************************************
 * 
 * Developed by:
 * 
 * Open Systems Lab
 * University of Illinois at Urbana-Champaign
 * Department of Computer Science
 * 201 North Goodwin Avenue
 * Urbana, IL  61801
 * http://osl.cs.uiuc.edu/
 * 
 * Smart Structures Technology Laboratory
 * University of Illinois at Urbana-Champaign
 * Department of Civil and Environmental Engineering
 * 205 North Matthews Avenue
 * Urbana, IL  61801
 * http://sstl.cee.uiuc.edu/
 * 
 * 
 * Send comments to: agha@cs.uiuc.edu
 * 
 * 
 * Copyright (c) 2008
 * The University of Illinois Board of Trustees.
 *      All Rights Reserved.
 * 
 * 
 * Principal Investigators:
 *      Gul A. Agha (agha@cs.uiuc.edu)
 * 	 B.F. Spencer, Jr. (bfs@uiuc.edu)
 * 
 * This work was supported in part by:
 * 
 * NSF grants CMS 06-00433 and CNS 05-09321 
 * NCASSR grant N00014-04-1-0562
 * DARPA grant F33615-01C-1907
 */

#ifndef GATEWAY
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "ResampleUSF.h"
#include <Xnode.h>


#define max_int(a, b)	((a) < (b) ? (b) : (a))
#define min_int(a, b)	((a) > (b) ? (b) : (a))
#define ceil_int(a, b)	((a)%(b)==0 ? (a)/(b) : (a)/(b)>=0 ? (a)/(b)+1 : (a)/(b))


int8_t ovlp_size_func_USF(int32_t Length, int32_t L, int32_t M)
{
	/*needs to be revised considering initial delay*/
	return (Length/2 + 1)/(L*((double)M/L - 1)) + 4 + 1;
}

void filter_InitDelayLI_USF(double fs1, double fs2, int32_t *y, uint32_t y_length, int16_t *filter_b, uint16_t filter_b_length, int32_t *z, uint32_t out_length, double IDr, uint8_t USF, uint8_t FilterSclFctr2)
{
/*	fs1:  	original sampling rate
	fs2:  	sampling rate after resampling
	y:		array of input signal. 
  z:	  array of the resampled signal
	filter_b:	An array of filter coefficients (latter half)
	filter_b_length:	The length of the filter
	out_length:	The length of resampled signal.
	ovlp_size:	The length of array which needs to store the resampled signal temporary to avoid overwritting the original signal to be used in the resampling. this is calculated by ovlp_size_f.
	IDr: 	Initial Delay in seconds.
	USF: 	Upsampling factor. 50-150 is test.
	FilterSclFctr2: Filter Scaling Factor. The filter coefficients are stored in 16-bit integer. When the original coefficients are b, then the stored filter coefficients are floor(b*2^FilterSclFctr2+0.5).
*/
/*
	Consistent with the thesis, T. Nagayama "Structural Health Monitoring Using Smart Sensors" Doctoral Dissertation, University of Illinois at Urbana-Champaign 2007.  
*/

/* 
	Author: 	Tomonori Nagayama 2007/12/01 
	contact:	nagayama@bridge.t.u-tokyo.ac.jp
*/

	int32_t L = USF, N = filter_b_length;
	double GD = (N - 1) / 2.; 
	double M;
	int32_t tmp;
	double yl, yu;
	int32_t j1, m, m1,m2;
	//int16_t *ovlp_y;
	double pj;
	int32_t pu, pl, pt;
	int64_t AveY = 0;
	int64_t y64u, y64l;
	filter_b[(int32_t)GD] = 0;
	
	//ovlp_y = (int16_t*)sdcalloc(ovlp_size, sizeof(int16_t)); configASSERT(ovlp_y);

	for (j1 = 0; j1 <= y_length-1; j1++) {
		AveY += y[j1];
	}
	AveY = AveY / y_length;
	for (j1 = 0; j1 <= y_length-1; j1++) {
		y[j1] -= AveY;
	}

	yu = 1. / ((int32_t)1 << FilterSclFctr2);
	M = fs1 * L / fs2;
	GD = GD - IDr * (fs1 * L);
	
	if ((filter_b_length % 2) == 0) {
		for (j1 = 0; j1 <= out_length - 1; j1++) {
			pj = j1 * M + GD;
			pl = (int32_t)floor(pj);
			pu = pl +1;
			y64l = 0;
			m1 = max_int((int32_t)(ceil_int((pl-N+1),L)),0);
			m2 = min_int((int32_t)(pl/L), y_length-1);
			for (m=m1;m<=m2;m++){
				tmp = pl-L*m-N/2;
				if (tmp>=0){
					y64l += (int64_t)filter_b[tmp]*y[m];  
				}else{
					y64l += (int64_t)filter_b[-tmp-1]*y[m]; 
				}
			}
			y64u = 0;
			m1 = max_int((int32_t)(ceil_int((pu-N+1),L)),0);
			m2 = min_int((int32_t)(pu/L), y_length-1);
			for (m=m1;m<=m2;m++){
				tmp = pu-L*m-N/2;
				if (tmp>=0){
					y64u += (int64_t)filter_b[tmp]*y[m];
				}else{
					y64u += (int64_t)filter_b[-tmp-1]*y[m];
				}
			}
			yl = (y64l*(pu-pj)+y64u*(pj-pl))/((int32_t)1<<FilterSclFctr2);
			z[j1] = (int32_t)floor(yl*L+0.5)+AveY;
			/*
			if (j1 <=ovlp_size-1){
				ovlp_y[j1] = (int16_t)floor(yl*L+0.5)+AveY;
				//z[j1] = (int16_t)floor(yl*L+0.5)+AveY;
			}else if (j1 < y_length){
				y[j1] = (int16_t)floor(yl*L+0.5)+AveY;
				//z[j1] = (int16_t)floor(yl*L+0.5)+AveY;
			}
			*/
		}
		/*
		for (j1 = 0;j1<=ovlp_size-1;j1++){
			//y[j1] = ovlp_y[j1];
			z[j1] = ovlp_y[j1];
		}
		*/
	}else{
		for (j1 =0;j1<=out_length-1;j1++){
			pj = (double)j1*M+GD;
			pl = (int32_t)floor(pj);
			pu = pl +1;
			y64l = 0;
			m1 = max_int((int32_t)(ceil_int((pl-N+1),L)),0);
			m2 = min_int((int32_t)(pl/L), y_length-1);
			pt = pl-N/2;
			for (m=m1;m<=m2;m++){
				tmp = pt-L*m;
				if (tmp>=0){
					y64l += (int64_t)filter_b[tmp]*y[m]; 
				}else{
					y64l += (int64_t)filter_b[-tmp]*y[m];
				}
			}
			y64u = 0.0;
			m1 = max_int((int32_t)(ceil_int((pu-N+1),L)),0);
			m2 = min_int((int32_t)(pu/L), y_length-1);
			pt = pu-N/2;
			for (m=m1;m<=m2;m++){
				tmp = pt-L*m;
				if (tmp>=0){
					y64u += (int64_t)filter_b[tmp]*y[m];
				}else{
					y64u += (int64_t)filter_b[-tmp]*y[m];
				}
			}
			yl = (y64l*(pu-pj)+y64u*(pj-pl))*yu;
			z[j1] = (int32_t)floor(yl*L+0.5)+AveY;

			/*
			if (j1 <=ovlp_size-1){
				ovlp_y[j1] = (int16_t)floor(yl*L+0.5)+AveY;
				//z[j1] = (int16_t)floor(yl*L+0.5)+AveY;
			}else if (j1 < y_length){
				y[j1] = (int16_t)floor(yl*L+0.5)+AveY;
				//z[j1] = (int16_t)floor(yl*L+0.5)+AveY;
			}
			*/
		}
		/*
		for (j1 = 0;j1<=ovlp_size-1;j1++){
			//y[j1] = ovlp_y[j1];
			z[j1] = ovlp_y[j1];
		}
		*/
	}
}


void filter_USF(double fs1, double fs2, int16_t *y, uint16_t y_length, int16_t *filter_b, uint16_t filter_b_length, int16_t *z, uint16_t out_length, uint8_t USF, uint8_t FilterSclFctr2)
{
	int32_t L = USF, N = filter_b_length;
	double GD = (N - 1) / 2.; 
	double M;
	int32_t tmp;
	double yl, yu;
	int32_t j1, m, m1,m2;
	//int16_t *ovlp_y;
	double pj;
	int32_t pu,pl, pt;
	int32_t AveY = 0;
	int64_t y64u, y64l;
	filter_b[(int32_t)GD] = 0;

	for (j1 = 0; j1 <= y_length-1; j1++) {
		AveY += y[j1];
	}
	AveY = AveY / y_length;
	for (j1 = 0; j1 <= y_length-1; j1++) {
		y[j1] -= AveY;
	}

	yu = 1. / ((int32_t)1 << FilterSclFctr2);
	M = fs1 * L / fs2;
	
	if ((filter_b_length % 2) == 0) {
		for (j1 = 0; j1 <= out_length - 1; j1++) {
			pj = j1 * M + GD;
			pl = (int32_t)floor(pj);
			pu = pl +1;
			y64l = 0;
			m1 = max_int((int32_t)(ceil_int((pl-N+1),L)),0);
			m2 = min_int((int32_t)(pl/L), y_length-1);
			for (m=m1;m<=m2;m++){
				tmp = pl-L*m-N/2;
				if (tmp>=0){
					y64l += (int64_t)filter_b[tmp]*y[m];  
				}else{
					y64l += (int64_t)filter_b[-tmp-1]*y[m]; 
				}
			}
			y64u = 0;
			m1 = max_int((int32_t)(ceil_int((pu-N+1),L)),0);
			m2 = min_int((int32_t)(pu/L), y_length-1);
			for (m=m1;m<=m2;m++){
				tmp = pu-L*m-N/2;
				if (tmp>=0){
					y64u += (int64_t)filter_b[tmp]*y[m];
				}else{
					y64u += (int64_t)filter_b[-tmp-1]*y[m];
				}
			}
			yl = (y64l*(pu-pj)+y64u*(pj-pl))/((int32_t)1<<FilterSclFctr2);
			z[j1] = (int16_t)floor(yl*L+0.5)+AveY;
		}
	}else{
		for (j1 =0;j1<=out_length-1;j1++){
			pj = (double)j1*M+GD;
			pl = (int32_t)floor(pj);
			pu = pl +1;
			y64l = 0;
			m1 = max_int((int32_t)(ceil_int((pl-N+1),L)),0);
			m2 = min_int((int32_t)(pl/L), y_length-1);
			pt = pl-N/2;
			for (m=m1;m<=m2;m++){
				tmp = pt-L*m;
				if (tmp>=0){
					y64l += (int64_t)filter_b[tmp]*y[m]; 
				}else{
					y64l += (int64_t)filter_b[-tmp]*y[m];
				}
			}
			y64u = 0.0;
			m1 = max_int((int32_t)(ceil_int((pu-N+1),L)),0);
			m2 = min_int((int32_t)(pu/L), y_length-1);
			pt = pu-N/2;
			for (m=m1;m<=m2;m++){
				tmp = pt-L*m;
				if (tmp>=0){
					y64u += (int64_t)filter_b[tmp]*y[m];
				}else{
					y64u += (int64_t)filter_b[-tmp]*y[m];
				}
			}
			yl = (y64l*(pu-pj)+y64u*(pj-pl))*yu;
			z[j1] = (int16_t)floor(yl*L+0.5)+AveY;
		}
	}
}
#endif

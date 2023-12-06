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

#ifndef _RESAMPLEUSF_H
#define _RESAMPLEUSF_H

#include <inttypes.h>
#include <LPC43xx.H>

int8_t ovlp_size_func_USF(int32_t Length, int32_t L, int32_t M);

void filter_InitDelayLI_USF(double fs1, double fs2, int32_t *y, uint32_t y_length, int16_t *filter_b, uint16_t filter_b_length, int32_t *z, uint32_t out_length, double IDr, uint8_t USF, uint8_t FilterSclFctr2);

/*	fs1:  	original sampling rate
	fs2:  	sampling rate after resampling
	y:		array of input signal. it will be replaced with the resampled signal
	Filter_b:	An array of filter coefficients (latter half)
	Filter_b_length:	The length of the filter
	out_length:	The length of resampled signal.
	ovlp_size:	The length of array which needs to store the resampled signal temporary to avoid overwritting the original signal to be used in the resampling. this is calculated by ovlp_size_f.
	ID: 	Initial Delay in seconds.
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

void filter_USF(double fs1, double fs2, int16_t *y, uint16_t y_length, int16_t *filter_b, uint16_t filter_b_length, int16_t *z, uint16_t out_length, uint8_t USF, uint8_t FilterSclFctr2);

#endif /* _RESAMPLEUSF_H */

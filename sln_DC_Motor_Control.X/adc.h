/* ADC converter functions
 *		
 *   *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author        	Date      			Comments on this revision		
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Serge Hould		December 15 2016   	-v1.1
 * Serge Hould		March 16, 22     	-v2	change function names
 * SH      October 2022         Add   #ifdef  SIMULATION 
 *
 *
 * 
 *
 **********************************************************************/
 
#ifndef __ADC_H_
#define __ADC_H_

#include <xc.h>

void adc_init(void);
#ifndef  SIMULATION
unsigned int adc_read( int );
#endif

#endif


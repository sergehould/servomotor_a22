/* 
 * File:   pv_measure.h
 * Author: sh
 *
 * SH               28 Jan 2021 
 * SH      October 2022         Add   #ifdef  SIMULATION 
 */

#ifndef PV_MEASURE_H
#define	PV_MEASURE_H
#include <stdint.h>

#ifndef  SIMULATION
int pv_get(void);
void pv_set(int pv);
#endif

void initPV_measure(void);

#endif	/* PV_MEASURE_H */


/* 
 * File:   DCMotor_mode.h
 * Author: sh
 *
 * Created on December 10, 2020, 7:49 AM
 *
 * SH      July 2022            Add   print_pos() and print_pwm()  
 * SH      October 2022         Add   #ifdef  SIMULATION 
 *           
*/  

#ifndef DCMOTOR_MODEL_H
#define	DCMOTOR_MODEL_H
#include "configuration.h"
#ifdef  SIMULATION
//int32_t pv_set(void);
void pv_set(int _pv);
int pv_get(void);
unsigned int adc_read( int);
void pwm_set(int);
#endif
void print_bar(int con, int val, char character, int centre, int range);
void plot_two_int32(int con, int val1, int val2, int centre, int range);
void plot_three_int32(int con, int val1, int val2, int val3, int centre, int range);


#endif	/* DCMOTOR_MODE_H */


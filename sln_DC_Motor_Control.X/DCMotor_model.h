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

#ifdef  SIMULATION
int32_t get_pv(void);
unsigned int adc_read( int);
void set_pwm(int);
#endif
void print_bar(int con, int val, char character, int centre, int range);

#endif	/* DCMOTOR_MODE_H */


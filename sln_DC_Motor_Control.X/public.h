/* 
 * File:   public.h
 * Author: sh
 *
 * Created on December 10, 2020, 3:10 PM
 */

#ifndef PUBLIC_H
#define	PUBLIC_H
#include <stdint.h>
#define LED7            LATAbits.LATA4
void ol_control(void);
void pd_loop(void);
void h_pid(int sp);
int h_knob(void);
int h_knob_v2(void);
int h_knob_v3(void);
int h_knob_sim(void);
int h_knob_lecture(void);
void haptic_pd();
//void response_print_pos(int console, char* txt, int range, int val, int time);
//void response_print_pos_neg(int console, char* txt, int range, int scale, int val, int time);
#endif	/* PUBLIC_H */


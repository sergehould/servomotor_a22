/* 
 * File:   tone.h
 * Author: sh
 *
 * Created on September 22, 2021, 5:59 PM
 * Serge Hould      28 Feb. 2022     Add heartbeat_init macro
 */

#ifndef UTIL_H
#define	UTIL_H

#ifdef	__cplusplus
extern "C" {
#endif
#define 	heartbeat_init  init_heartbeat
#define		tone_init			init_tone

void heartbeat(void);
void init_heartbeat(void);
void tone_high(int cnt);
void tone_low(int cnt);
void init_tone(void);
void delay_ms( unsigned int);
void delay_10us( unsigned int);
long map(long x, long in_min, long in_max, long out_min, long out_max);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);



#ifdef	__cplusplus
}
#endif

#endif	/* TONE_H */


/* 
 * File:   configuration.h
 * Description:
 *  Includes all common macros needed by many files
 *  Includes the important macro to swap from simulation mode to target mode and vis-versa.
 * 
 * Author: sh
 *
 * SH   28 Jan 2021     v1.0             
 */

#ifndef CONFIGURATION_H
#define	CONFIGURATION_H
/* Macro to swap from simulation mode to target mode and vis-versa */
/* To run target mode, comment out this line.                        */
//#define SIMULATION 

#define DIR             LATGbits.LATG9  //dir
#define HUNDRED_DUTY    10000   //10000 uS or 100 Hz
#define REVERSE         0
#define FORWARD         1

#endif	/* CONFIGURATION_H */


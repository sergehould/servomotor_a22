/******************************************************************************
 * FileName:	main.c 
 *	
 * Name:	Lab# 
 *
 * Description: To complete ... 
 *  
 * REVISION HISTORY:
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author  	Date                	Comments on this revision
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * To complete...
 *                                 
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "initBoard.h"
#include "console.h"
#include "DCMotor_model.h"
#include "pwm.h"
#include "pv_measure.h"
#include "configuration.h"
#include "adc.h"

#define LED3    LATAbits.LATA0



int main(void){

    /**** Initializes UARTs *****/
    uart1_init();
    uart2_init();;  
    stdio_set(C_UART1);
    printf("Board reset1\n");
    stdio_set(C_UART2);
    printf("Board reset2\n");
   
    /* Super loop */
	while (1){

    }   
}
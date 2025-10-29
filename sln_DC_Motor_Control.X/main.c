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
#include "BDCM_sim.h"
#include "pwm.h"
#include "pv_measure.h"
#include "configuration.h"
#include "adc.h"
#include "public.h"

#define LED3    LATAbits.LATA0


int main(void){
    /**** Initializes resources *****/
    uart2_init(115200);
    io_init();
    PMOD_init();
    initPWM();
    initPV_measure();
    adc_init();
    lcd_init();

    //stdio_set(C_UART2);
    stdio_set(C_LCD);
    printf("\n\rLine1   ");
    printf("\n\r\nLine2   ");
   
    /* Super loop */
	while (1){
        ol_loop();
       
    }   
}
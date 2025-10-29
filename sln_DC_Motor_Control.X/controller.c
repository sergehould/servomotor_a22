/**************************************************************************** 
 * File name :   controller.c
 * 
 * Author           Date            V       Description
 * 
 ***************************************************************************/

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "initBoard.h"
#include "console.h"
#include "BDCM_sim.h"
#include "pwm.h"
#include "pv_measure.h"
#include "configuration.h"
#include "adc.h"
#include "public.h"
#include "Tick_core.h"

#define PERIOD  TICKS_PER_SECOND / 100      //10mS

/**************************************************************************** 
 * NAME :   ol_loop
 * 
 * DESCRIPTION :    The open loop controller for the DC motor.
 ***************************************************************************/
void ol_loop (void) {
    static int stamp = 0;
    
    /* Executes every 10 mS */
    if(tick_diff(stamp) > PERIOD) {
        stamp = tick_get(); //restamp
        
    }
}

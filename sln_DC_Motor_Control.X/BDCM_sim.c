/* DCMotor_model_elect_mechan.c     
 * 
 * Simplified version
 * 
 * Using DMCI one-time-write features can clear pv,accel and speed at once
 * WORKS PERFECTLY AND LIVE - NO NEED TO PAUSE THE DEBUGGER
 * 
 * This model works at 100Hz (same as real motor) and important variables are ints 
 * rather than floats. The tic granularity is 1 or 2 maximum allowing to simulate 
 * the gerbox-less motor.
 *      
 *  SH      12 Nov. 2025    pwm_set() passes float value from -100.00 to +100.00
 * 
 * */                                                 

#include <math.h>
#include <xc.h>
#include <stdio.h>
#include "../../common/console.h"
#include "console_print.h"
#include "BDCM_sim.h"
#include <stdint.h>
#include <string.h>
#include "configuration.h"
#include "public.h"


/* UART BUG*/
/*
 * During simulation when printing to UARTs, we sometime get the following 
 * infamous exception message from the simulator:
 *  W0011-CORE: Access attempt to unimplemented RAM memory.
 *  Simulator halted
 * 
 * In order to continue running, variable have been set persistent so if 
 * RUN is pressed again, variables won't get reset.
 * Just keep running by pressing RUN again. (DO NOT Restart or Reset)
 */
// global to transmit button values
int set_pv_10=0;
int set_pv_5=0;
int set_pv_minus_10=0;
int set_pv_minus_5=0;
int set_pv_25=0;
int set_pv_minus_25=0;

static int time __attribute__((persistent));
//int pv1 __attribute__((persistent)); //pv value
//int accel; // accel is the speed slope
//int speed=0; // motor speed
double pv1 __attribute__((persistent)); //pv value
double accel; // accel is the speed slope
double speed=0; // motor speed
//int adc_sim=1280;
int adc_sim=16; // adc value in simulation mode


#ifdef  SIMULATION

static void response_print_pos_neg2(int console, int range, int scale, int zero, int val, char character, int time) ;

/********************************************************
  Model for a BDC motor                              
    bemf = Ks * speed
    vm = Vref- bemf  
    accel = vm/J = vm* _J  where _j = 1/J
    speed = accel*DT + speed
    pv = pv + speed * DT        
                                                      
*********************************************************/

#define     KT      0.02       // motor torque constant in N.m/A
#define     KS      0.01       // EMF constant V/rad/sec
#define 	_J		1000           // one over rotor's moment of inertia: 1/J for haptic_knob
//#define 	_J		3000           // Greater acceleration
#define     DT      0.001  // dt time increase - a tad too tight 

/* Overhead is pretty tight: 50uS */
//void pwm_set(int pwm){
//    int _pwm=0;
//    /* Electrical */
//    double vm; // voltage across the motor 
//    double vref; // pwm converted into volts
//    if(pwm >HUNDRED_DUTY-1   ) _pwm = HUNDRED_DUTY-1  ;
//    else if(pwm  < -HUNDRED_DUTY+1 ) _pwm = -HUNDRED_DUTY+1;
//    else _pwm =pwm;
//    time++;
//    vref =(double)_pwm/20;  
//    vm = vref-KS*speed; // vm = vref -bemf = vref - (Ks * speed)
//    accel = vm*_J;
//    speed = accel*DT + speed; 
//    pv1 = pv1+ speed*DT;
//
//    if(set_pv_5 ==1){
//            pv1= 5;
//            speed=0;
//            accel=0;
//    }
//    else if (set_pv_10 ==1){
//            pv1= 10;
//            speed=0;
//            accel=0;
//    }
//    else if (set_pv_minus_10 ==1){
//            pv1= -10;
//            speed=0;
//            accel=0;
//    }
//    else if (set_pv_minus_5 ==1){
//            pv1= -5;
//            speed=0;
//            accel=0;
//    }
//    else if (set_pv_25 ==1){
//            pv1= 25;
//            speed=0;
//            accel=0;
//    }
//    else if (set_pv_minus_25 ==1){
//            pv1= -25;
//            speed=0;
//            accel=0;
//    }
//
// }

void pwm_set(float pwm_f){
    int pwm = (int)pwm_f*100; // float to int conversion
    int _pwm=0;
    /* Electrical */
    double vm; // voltage across the motor 
    double vref; // pwm converted into volts
    if(pwm >HUNDRED_DUTY-1   ) _pwm = HUNDRED_DUTY-1  ;
    else if(pwm  < -HUNDRED_DUTY+1 ) _pwm = -HUNDRED_DUTY+1;
    else _pwm =pwm;
    time++;
    vref =(double)_pwm/20;  
    vm = vref-KS*speed; // vm = vref -bemf = vref - (Ks * speed)
    accel = vm*_J;
    speed = accel*DT + speed; 
    pv1 = pv1+ speed*DT;

    if(set_pv_5 ==1){
            pv1= 5;
            speed=0;
            accel=0;
    }
    else if (set_pv_10 ==1){
            pv1= 10;
            speed=0;
            accel=0;
    }
    else if (set_pv_minus_10 ==1){
            pv1= -10;
            speed=0;
            accel=0;
    }
    else if (set_pv_minus_5 ==1){
            pv1= -5;
            speed=0;
            accel=0;
    }
    else if (set_pv_25 ==1){
            pv1= 25;
            speed=0;
            accel=0;
    }
    else if (set_pv_minus_25 ==1){
            pv1= -25;
            speed=0;
            accel=0;
    }

 }
/* In simulation mode, get_pv() and reaADC() are provided by BDCM_sim.c */
/* In target mode, get_pv() and reaADC() are provided respectively by pv_measure.c and adc32.c */
int pv_get(void){    
    if(set_pv_5 ==1){
            pv1= 5;
            speed=0;
            accel=0;
    }
    else if (set_pv_10 ==1){
            pv1= 10;
            speed=0;
            accel=0;
    }
    else if (set_pv_minus_10 ==1){
            pv1= -10;
            speed=0;
            accel=0;
    }
    else if (set_pv_minus_5 ==1){
            pv1= -5;
            speed=0;
            accel=0;
    }
    else if (set_pv_minus_25 ==1){
            pv1= -25;
            speed=0;
            accel=0;
    }
        else if (set_pv_25 ==1){
            pv1= 25;
            speed=0;
            accel=0;
    }
    return pv1;
}

void pv_set(int _pv){
    pv1= _pv;
    speed=0;
    accel=0;
}


unsigned int adc_read( int ch){
    unsigned int adc;
    adc = adc_sim;
    return adc;
}

void adc_init( void){

}

/* Function to plot positive and negative values*/
/* It keeps the bargraph within the limits */
static void response_print_pos_neg2(int console, int range, int scale, int zero, int val, char character, int time) {
    int j, half_scale, lim, max, val_print;
    val_print = val;
    // static int time=0;
    char buff[100], buff2[100];
    if (val > range) val = range;
    else if (val < -range) val = -range;
    half_scale = scale / 2;
    /* Prints time line */
    if(time != -1){
        sprintf(buff, "%4dmS ", time*10);
    }
    else sprintf(buff, "       ");
    fprintf2(console, buff);
    sprintf(buff, "%3d ", val_print);
  

    if (val < 0) {
        /* Prints negative plot scaled down val to a specific range */
        lim = (half_scale * zero / 50 + (val * scale) / range) - strlen(buff) - 6; // 6 is the time stamp 
        if (lim > 0) {
            for (j = 0; j < lim; j++) {
                //printf(" ");
                fprintf2(console, " ");
            }
            for (j = 0; j < (-val * half_scale) / range; j++) {
                //printf("%c", character);
                sprintf(buff2,"%c", character);
                fprintf2(console, buff2);
            }
            //printf(buff);
            fprintf2(console, buff);
            for (j = 0; j < (-val * half_scale) / range; j++) {
                //printf("%c", character);
                sprintf(buff2,"%c", character);
                fprintf2(console, buff2);
            }
            //printf("|");
            fprintf2(console, "|");
        }
        else { // if exceeds negative screen, cap it
            //printf(" ");
            fprintf2(console, " ");
            //printf(buff);
            fprintf2(console, buff);
            for (j = 0; j < half_scale * zero / 50 - 11; j++) {
                //printf("%c", character);
                sprintf(buff2,"%c", character);
                fprintf2(console, buff2);
            }
            //printf("|");
            fprintf2(console, "|");
        }
    }
    /* Prints positive plot scaled down val to a specific range */
    else {
        /* Prints ... */
        //for (j = 0; j < 77 - strlen(buff); j++) {
        max = (half_scale * zero / 50 + (val * scale) / range); // 6 is the time stamp 
        if (max < (scale)) { // enough space to print
            for (j = 0; j < half_scale * zero / 50 - 6; j++) { // 6 is the time stamp 
                //printf(" ");
                fprintf2(console, " ");
            }
            //printf("|");
            fprintf2(console, "|");
            for (j = 0; j < (val * half_scale) / range; j++) {
                  //printf("%c", character);
                  sprintf(buff2,"%c", character);
                  fprintf2(console, buff2);
            }
            sprintf(buff, " +%3d ", val_print);
            //printf(buff);
            fprintf2(console, buff);
            for (j = 0; j < (val * half_scale) / range; j++) {
                //printf("%c", character);
                sprintf(buff2,"%c", character);
                fprintf2(console, buff2);
            }
        }
        else { // if exceeds positive screen. cap it.
            for (j = 0; j < half_scale * zero / 50 - 6; j++) { // 6 is the time stamp 
                //printf(" ");
                fprintf2(console, " ");
            }
            //printf("|");
            fprintf2(console, "|");
            for (j = 0; j < (scale - half_scale * zero / 50); j++) {
                //printf("%c", character);
                sprintf(buff2,"%c", character);
                fprintf2(console, buff2);
            }
            sprintf(buff, " +%3d ", val_print);
            //printf(buff);
            fprintf2(console, buff);

        }
    }
    //printf("\n\r");
    fprintf2(console, "\n\r");
}



/* void print_bar(int con, int val, char charact, int centre, int range)
 * 
 * Description: Function to print in a bar graph format to the specified 
 *              serial console.
 * 
 * Parameters:
 *      con: specifies the console (e.g. C_UART1, C_UART2).
 *      val: specifies the value to display as a bar graph.
 *      charact: specifies the ASCII character forming the bar graph (e.g. '*').
 *      centre: specifies the horizontal position of the zero for the bar graph. 
 *              The range of values are 0% to 100% (e.g. 50 for 50%).
 *      range: specifies the range of the bar graph (e.g. 1000).
 *      Usage example#1:
 *          The following line prints pv as a bar graph using a star character.
 *          The bar graph zero is at the left side of the screen (0%). 
 *          The range is 1000 (0 to 1000 in this case). 
 *          print_bar(C_UART2, get_pv(), '*', 0 , 1000 ); 
 * 
  *      Usage example#2:
 *          The following line prints pwm as a bar graph using a 'w' character.
 *          The bar graph zero is in the center of the screen (50%). 
 *          The range is 20000 (-10000 to 10000 in this case). 
 *          print_bar(C_UART2, pwm, 'w', 50 , 20000 ); 
 *      
*/

void print_bar(int con, int val, char character, int centre, int range){
     if(time%25 == 0){
        if(val > 40000) val = 40000;
        else if (val < -40000) val = -40000;
        if(centre> 100) centre = 100;
        else if(centre < 0) centre = 0;
        if(range> 80000) range = 80000;
        else if(range < 0) range = 0;
        //void response_print_pos_neg2(int console, int range, int scale, int zero, int val, char character, int time) 
         response_print_pos_neg2(con, range, 110, centre, val, character, time/5);
     }
    
}

void plot_two_int32(int con, int val1, int val2, int centre, int range){
        if(val1 > 40000) val1 = 40000;
        else if (val1 < -40000) val1 = -40000;
        if(val2 > 40000) val2 = 40000;
        else if (val2 < -40000) val2 = -40000;        
        if(centre> 100) centre = 100;
        else if(centre < 0) centre = 0;
        if(range> 80000) range = 80000;
        else if(range < 0) range = 0;
        //void response_print_pos_neg2(int console, int range, int scale, int zero, int val, char character, int time) 
         response_print_pos_neg2(con, range, 110, centre, val1, '@', time);
         response_print_pos_neg2(con, range, 110, centre, val2, '*', time);
}

void plot_three_int32(int con, int val1, int val2, int val3, int centre, int range){
        if(val1 > 40000) val1 = 40000;
        else if (val1 < -40000) val1 = -40000;
        if(val2 > 40000) val2 = 40000;
        else if (val2 < -40000) val2 = -40000; 
         if(val3 > 40000) val3 = 40000;
        else if (val3 < -40000) val3 = -40000;  
        if(centre> 100) centre = 100;
        else if(centre < 0) centre = 0;
        if(range> 80000) range = 80000;
        else if(range < 0) range = 0;
        //void response_print_pos_neg2(int console, int range, int scale, int zero, int val, char character, int time) 
         response_print_pos_neg2(con, range, 110, centre, val1, '@', time);
         response_print_pos_neg2(con, range, 110, centre, val2, '*', -1);
         response_print_pos_neg2(con, range, 110, centre, val3, '#', -1);
}

#else


void print_bar(int con, int val, char character, int centre, int range){
        char buff2[100];
        if(val > 40000) val = 40000;
        else if (val < -40000) val = -40000;
        if(centre> 100) centre = 100;
        else if(centre < 0) centre = 0;
        if(range> 80000) range = 80000;
        else if(range < 0) range = 0;
        sprintf(buff2,"%5d+%5d+%5d%c\n",val, centre,range, character); 
        fprintf2(con, buff2); // dump to serial program receiver - see MPLAB_projects\projects\DC_Motor_Ctl\project_visual_studio\serial
     
}

/* Description: Function to plot two int32 variables to the specified  
 *              serial console. 
 *  
 * Parameters: 
 *      con: specifies the console (e.g. C_UART1, C_UART2). 
 *      val1: specifies the first value to display.
 *      val2: specifies the second value to display  
 *      center: specifies the horizontal position of the zero for the bar graph.  
 *              The range of values are 0% to 100% (e.g. 50 for 50%). 
 *      range: specifies the range of the bar graph (e.g. 1000). 
 * 
 *      Usage example#1: 
 *          The following line prints pv and adc as two bar graphs.
 *          The bar graph zero is at the left side of the screen (0%).  
 *          The range is 1000 (0 to 1000 in this case).  
 *          print_bar(C_UART2, get_pv(), adc, 0 , 1000 );  
 *  
 *      Usage example#2: 
 *          The following line prints pv and pwm as two bar graphs. 
 *          The bar graph zero is in the center of the screen (50%).  
 *          The range is 20000 (-10000 to 10000 in this case).  
 *          print_bar(C_UART2, pv, pwm, 50 , 20000 );      
*/ 
void plot_two_int32(int con, int val1, int val2, int centre, int range){
    send_two_int32(val1,val2);
}
#endif


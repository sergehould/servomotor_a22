/* DCMotor_model_elect_mechan.c     
 * Improved mathematical model representing a BDC motor    
 * 
 *  Author  Date                    Comments/version 
 *  SH      11 July 2022            V1.0    
 *  SH      16 Aug. 2022            V2.0    print_bar() to print both simulation and target mode
 *                                          It uses response_print_pos_neg2() see also VStudio code
 * 
 * 
 * */                                                 

/*
 Real motor measured characteristic table:

 */
#include <math.h>
#include <xc.h>
#include <stdio.h>
#include "console.h"
//#include "console_print.h"
#include "DCMotor_model.h"
#include <stdint.h>
#include <string.h>
#include "configuration.h"


/* Globals */

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

//static double i __attribute__((persistent));
static double ITerm2 __attribute__((persistent));
static int16_t ITerm2_int16 __attribute__((persistent));
static int time __attribute__((persistent));
static int pos __attribute__((persistent));
//int adc_sim=1280;
int adc_sim=500;
static double speed=0; // motor speed


/********************************************************
  Model for a BDC motor                              
  Motor gain 50 RPM/pwm                               
      E.g.    pwm =50% -> speed = 2500 RPM            
              pwm =100% -> speed = 5000 RPM           
                                                      
 T = Kt*i
 bemf = Ks * speed
 
 Vs- bemf = Ldi/dt + Ri
 J * d2angle/dt + B * dangle/dt = Kt*i
*********************************************************/
#ifdef  SIMULATION
static void response_print_pos_neg2(int console, int range, int scale, int zero, int val, char character, int time) ;
static double static_friction(double ki, double speed);

#define		L		0.001    // Inductance of the motor
#define		R		2       // Resistance of the motor
#define     KT      0.01       // motor torque constant in N.m/A
#define     J       0.00001       // rotor's moment of inertia
#define     KS      0.01       // EMF constant V/rad/sec
#define     B       0.000004       // motor viscous friction 

#define     DT      0.0001  // dt time increase - a tad too tight 
//#define     DT      0.0002  // dt time increase - less tight but must not be too large
// static friction parameters 
#define     OFFSET		0.01
#define     THRESHOLD	0.15

/* Overhead is pretty tight: 50uS */
void set_pwm(int pwm){
    int debug2;
    double debug3;
    static int disturb_cnt=0,disturb_cnt2=0, disturb_f=0;
    int _pwm=0;
    float disturb_i=0;
    
    /* Electrical */
	double vl; // Vl is the inductance voltage
    double didt; // didt is the current slope
    static double i=0; // motor current
    double vm; // voltage across the motor 
    double vref; // pwm converted into volts
    
    /* Mechanical */
    double kti; //Kt*i

    double dsdt; // dsdt is the speed slope
    double vj    ; // like a voltage across J
    static double position; // shaft position
    static int pol =1;
    double position_tics;


    if(pwm >HUNDRED_DUTY-1   ) _pwm = HUNDRED_DUTY-1  ;
    else if(pwm  < -HUNDRED_DUTY+1 ) _pwm = -HUNDRED_DUTY+1;
    else _pwm =pwm;
    time++;
    debug2 =i;
    /* Motor RL circuit */
    vref =(double)_pwm/1000;     // must divide by 1000 if pwm of 10000 is 10V. In mA, must multiply by 1000. They cancel each other.
    //vref /= 6;                  // To compensate for the discrepancy between the model and the real motor
    vm = vref-KS*speed;

    /* without inductance- simplified */
    i = vm/R;
    /* Static friction */
    i = static_friction( i, speed);
    
    /* Motor mechanical */
    kti = KT *i;
    vj = kti - B*speed;
    dsdt = vj/J;
    speed = dsdt*DT + speed;
    debug2 =speed;
    position = position+ speed *DT; // position in rad
    position_tics = position*203.7; // position in tics
    
    pos = position_tics;  // for display
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
    //sprintf(buff,"%4dmS ",time*10);
    sprintf(buff, "%4dmS ", time);
    //printf(buff);
    fprintf2(console, buff);

    sprintf(buff, "%3d", val_print);

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
            sprintf(buff, "%3d", val_print);
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
            sprintf(buff, "%3d", val_print);
            //printf(buff);
            fprintf2(console, buff);

        }
    }
    //printf("\n\r");
    fprintf2(console, "\n\r");
}

/* In simulation mode, get_pv() and reaADC() are provided by DCMotor_mode_xxx.c */
/* In target mode, get_pv() and reaADC() are provided respectively by pv_measure.c and adc32.c */

int32_t get_pv(void){
        return pos;
}

//int readADC( int ch){
//    long adc;
//    adc = adc_sim;
//    adc = adc_sim + 0x8000;
//    adc = ((long)adc*1024/0x10000);
//    return adc;
//}

unsigned int adc_read( int ch){
    unsigned int adc;
    adc = adc_sim;
    return adc;
}

void adc_init( void){

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

double static_friction(double _i, double speed){
    if(fabs(speed)>OFFSET)return _i;
	else{
		if(fabs(_i) < THRESHOLD) return 0;
		else return _i;
	}
}

/* Must be called at a constant period. e.g. every 10 mS*/
/* Reads the speed in RPM */
int get_speed(void){
    return speed;
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


#endif
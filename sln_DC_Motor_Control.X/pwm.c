/*
 *		pwm.c
 * 
 *      Timer3 ISR to set the pwm
 * 
 *		Serge Hould     Jan  2021       v1.0   
 *      Serge Hould     27 Jan  2021    v1.1		The T3ClearIntFlag() was moved. 
 *                                                  See comments below.
 *      SH              8  Feb 2021     v1.2        Got rid of peripheral lib. Replace INTEnableSystemMultiVectoredInt()
 *                                                  by INTCONbits.MVEC=1; No more library needed.  Needs only sys/attribs.h
 * 
 *      SH              12 Nov. 2025                pwm_set() passes float value from -100.00 to +100.00
 */
#include <xc.h>
#include "pwm.h"
#include "configuration.h"
#include <sys/attribs.h>
static int32_t on_time2=0;

/* Initializes the Timer3 and the ISR to implement a soft PWM */
/*  init timer 3  to 40MHz/(16*10000) = 250Hz or 4mS */
void initPWM(void){
#ifndef  SIMULATION
    PR3 = HUNDRED_DUTY;
    /*
        111 = 1:256 prescale value
        110 = 1:64 prescale value
        101 = 1:32 prescale value
        100 = 1:16 prescale value
        011 = 1:8 prescale value
        010 = 1:4 prescale value
        001 = 1:2 prescale value
        000 = 1:1 prescale value
     */
    //T3CONbits.TCKPS = 0b101; // set prescaler 1:32
    T3CONbits.TCKPS = 0b100; // set prescaler 1:16
    //T3CONbits.TCKPS = 0b000; // set prescaler 1:1
    T3CONbits.TGATE = 0; // not gated input (the default)
    T3CONbits.TCS = 0; // PCBLK input (the default)
    INTCONbits.MVEC=1; // enable multiVectoredInt
    
    __builtin_disable_interrupts(); // disable interrupts, remember initial state
    //init interrupts
    IPC3bits.T3IP = 1; // Interrupt priority  (low)
    IFS0bits.T3IF = 1;
    IEC0bits.T3IE = 1;
    IFS0bits.T3IF=0;
    
    T3CONbits.ON = 1; // turn on Timer
     __builtin_enable_interrupts();

    
#endif
}

void __ISR( _TIMER_3_VECTOR, IPL1SOFT) T3InterruptHandler( void){
    static int on_f=0;
    /* The flag needs to be cleared here otherwise when TMR3 is set to zero
     it will overflow and clear the flag when it reaches the end of the ISR*/
    //mT3ClearIntFlag(); 
    IFS0bits.T3IF=0;
    if(on_f){
        TMR3 = on_time2;
        //TMR3 = test;
        on_f = 0;
        EN = 0;
    }
    else{
        TMR3 = HUNDRED_DUTY - on_time2;
        //TMR3 = PR3 - test;
        on_f = 1;
        EN = 1;
    }

} // T3 Interrupt Handler

#ifndef  SIMULATION
/* Sets the PWM. 			
 * The parameter takes a range from -10000 to +10000
 * A negative value represents a CCW direction
 * and a positive value represents a CW direction.
 * A value of 10000 represents a duty cycle of 100.00% CW direction
 * A value of 0 represents a duty cycle of 0.00%
 * A value of -10000 represents a duty cycle of 100.00% CCW direction
 */
//void pwm_set(int on){
//            if(on<0){
//                //DIR = FORWARD;
//                DIR = REVERSE;
//                on_time2 = -on;
//            }
//            else{
//                DIR = FORWARD;
//                on_time2 = on;
//                //DIR = REVERSE;
//            }
//            /******* Caps the pwm value **********/
//            if(on_time2 >HUNDRED_DUTY-1   ) on_time2 = HUNDRED_DUTY-1  ;
//            else if(on_time2  < 0 ) on_time2=0;
//}
void pwm_set(float on_f){
            int on = (int)on_f*100; // float to int conversion
            if(on<0){
                //DIR = FORWARD;
                DIR = REVERSE;
                on_time2 = -on;
            }
            else{
                DIR = FORWARD;
                on_time2 = on;
                //DIR = REVERSE;
            }
            /******* Caps the pwm value **********/
            if(on_time2 >HUNDRED_DUTY-1   ) on_time2 = HUNDRED_DUTY-1  ;
            else if(on_time2  < 0 ) on_time2=0;
}
#endif

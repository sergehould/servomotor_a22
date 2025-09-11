/*
 *		pv_measure.c
 * 
 *      Measure the position of the motor by reading the rotary encoder pulses.
 *      Implemented using an CN interrupt.
 * 
 *		Serge Hould     Jan  2021       v1.0    		
 *      Serge Hould     3 Feb 2021      v1.1    simpify code using register only
 *                                              No PLIB functions    
 *      Serge Hould     15 June 2022    v2.0    Use a state machine to increase 
 *                                              by 4 the control resolution of the encoder. 
 *                                              Control resolution (CR) on the gearbox side: 3.555 tics/degree (or 1280 tics/rev)
 *      Serge Hould     12 July 2022    v2.1    Debugged get_speed()
 */ 


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/attribs.h>
//#include "peripheral/int.h"
#include "pv_measure.h"
#include "configuration.h"

#define     ENCA        _RG7          
#define     ENCB        _RG6          

static inline int encState(void);
static void encoderTask(void);
static void encoderTask2(void);

static int32_t pos=0;  // motor position

// change notification interrupt service routine
//void __ISR( _CHANGE_NOTICE_VECTOR, IPL1SOFT) CNInterrupt( void){ 
//    
//    if(PORTGbits.RG7){
//        if(PORTGbits.RG6) pos++;
//        else pos--;
//    }
//    // clear interrupt flag
//    IFS1bits.CNIF=0;   
//}
// change notification interrupt service routine
void __ISR( _CHANGE_NOTICE_VECTOR, IPL1SOFT) CNInterrupt( void){ 
    
    encoderTask();
    // clear interrupt flag
    IFS1bits.CNIF=0;   
}

/* returns the motor position in tics */

#ifndef  SIMULATION
/* In simulation mode, get_pv() is provided by DCMotor_model3.c */
/* In target mode, get_pv() is provided respectively by pv_measure.c */
int pv_get(void){
    return pos;
}

void pv_set(int pv){
    pos =pv;
}

/* Must be called at a constant period. e.g. every 10 mS*/
/* Reads the speed in RPM */
int get_speed(void){
    static int pos_mem;
    int period, speed;
    if(DIR == FORWARD)period = pos-pos_mem;
    //if(pos) period = pos-pos_mem;
    else period = pos_mem-pos;
    pos_mem = pos;
    speed = ((long)60* period *100)/64; // speed in RPM. Multiply by 100 for 1 second.
    return speed;
//    if(speed != 0){
//        speed = 2000/speed;
//        return speed;
//    }else return 0;
}
#endif



/* Initializes the CN interrupt */
void initPV_measure(void){
#ifndef  SIMULATION
    //CN
    CNENbits.CNEN9 = 1; // //SA channel- RG7
    CNENbits.CNEN8 = 1; //SB channeL - RG6
    INTCONbits.MVEC=1; // enable multiVectoredInt
    __builtin_disable_interrupts();   // disable interrupts
    CNCONbits.ON = 1; // turn on Change Notification
    IPC6bits.CNIP=1;  
    IFS1bits.CNIF=0;   
    IEC1bits.CNIE=1;
    __builtin_enable_interrupts();   // enable interrupts
#endif
}


void encoderTask(void) {
//	static unsigned long lastTick;
    static unsigned int shift = 0x1, debug1=0, switch_f =0;
    static int mem_state=0;
    //static int encState =0;
 	//static enum {SM_1, SM_2,SM_3,SM_4} state=0; //
    static enum {SM_10, SM_11,SM_01,SM_00} state=0; //
    //encState = ENCA << 1 | ENCB;
   // encShadow =encState;
    
    
    
    if(mem_state != encState()){
        debug1++;   // Put a break point here to debug the FSM
                    // It will stop on a new encoder state
    }
    //mem_state = encState();

   	switch(state){
		case SM_10://10
            if(encState() ==0b11 ){
                state = SM_11;
                pos--;
            }
            else if(encState() == 0b00){
                state = SM_00;
                pos++;  
            }
            else state = SM_10;
         	break;		
		case SM_11: //11
            if(encState() ==0b01 ){
                state = SM_01;
                pos--;
            }
            else if(encState() == 0b10){
                state = SM_10;
                pos++;
            }
            else state = SM_11;
         	break;	
		case SM_01: //01
            if(encState() ==0b00 ){
                state = SM_00;
                pos--;         
            }
            else if(encState() == 0b11){
                state = SM_11;
                pos++;     
            }
            else state = SM_01;
         	break;	
		case SM_00: //00
            if(encState() ==0b10 ){
                state = SM_10;
                pos--;
            }
            else if(encState() == 0b01){
                state = SM_01;
                pos++;
            }
            else state = SM_00;
         	break;	
	} // switch
}


/* Simplified version - tested ok */
void encoderTask2(void) {
    static enum {SM_10, SM_11,SM_01,SM_00} state=0; //
   	switch(state){
		case SM_10://10
            if(ENCB ==0b1 ){
                state = SM_11;
                pos--;
            }
             else if(ENCA ==0b0 ){
                state = SM_00;
                pos++;  
            }
            else state = SM_10;
         	break;		
		case SM_11: //11
            if(ENCA ==0b0 ){
                state = SM_01;
                pos--;
            }
            else if(ENCB == 0b0){
                state = SM_10;
                pos++;
            }
            else state = SM_11;
         	break;	
		case SM_01: //01
            if(ENCB == 0b0 ){
                state = SM_00;
                pos--;         
            }
            else if(ENCA == 0b1){
                state = SM_11;
                pos++;     
            }
            else state = SM_01;
         	break;	
		case SM_00: //00
            if(ENCA == 0b1 ){
                state = SM_10;
                pos--;
            }
            else if(ENCB == 0b1){
                state = SM_01;
                pos++;
            }
            else state = SM_00;
         	break;	
	} // switch
}

void setCnt(int cnt){
    pos= cnt;
}

int getCnt(void){
    return pos;
}



static inline int encState(void){
   return  ENCA << 1 | ENCB;
}

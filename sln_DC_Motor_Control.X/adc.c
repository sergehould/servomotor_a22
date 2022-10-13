
/* File: adc.c
 * 
 * ADC converter functions 
 *		
 *   *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author        	Date      			Comments on this revision		
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Serge Hould		December 15 2016    -v1.1
 * Serge Hould		14 Jan. 2021     	-v2.0 Modify for PIC32				
 *												Setup to POT as analog input pin by default 
 *												the channel is AN2 see note below
 * Serge Hould		March 16, 22     	-v3.0	Change function names
 *												Add MX3 and MICROSTICK_II boards
 *                                              Not tested yet
 * SH               April 3, 2022       -v3.1   Enable pin 24 and pin 21 as 
 *                                              analog - see adc_init()
 *
 *  
 *
 **********************************************************************/



#include "adc.h"
#include "configuration.h"

#if defined EXPLORER_16_32
// The PIM socket pin numbers is not  a 1 to 1 match to the PIC pins:
// According to the PIM datasheet pin 20 on the Explorer16/32 (the pin actually connected to the potmeter),
// connects to pin 23 on the device, which is AN2. 

#ifndef  SIMULATION
/* initialize the ADC for single conversion, select Analog input pins */
void adc_init(void) 
{
    AD1PCFGbits.PCFG1 = 0; // AN1 pin 24 is analog
    AD1PCFGbits.PCFG4 = 0; // AN4 pin 21 is analog
    AD1PCFGbits.PCFG2 = 0; // AN2 is an adc pin for the pot
    AD1CON3bits.ADCS = 2; // ADC clock period is Tad = 2*(ADCS+1)*Tpb =
    // 2*3*12.5ns = 75ns
    AD1CON1bits.ADON = 1; // turn on A/D converter
} //initADC
#endif

/**
  Prototype:        unsigned int adc_read( int ch)
  Input:            Channel to read
  Output:           returns the conversion result
  Description:      Select the specified input channel, samples the channel 
                    and then returns the converted value.
  Comment:          
  Usage:           
*/
#ifndef  SIMULATION
#define SAMPLE_TIME 10 // 10 core timer ticks = 250 ns
unsigned int adc_read( int ch)
{
    // adc pin the pin should be configured as an
    // analog input in AD1PCFG
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = ch; // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1; // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) {
    ; // sample for more than 250 ns
    }
    AD1CON1bits.SAMP = 0; // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
    ; // wait for the conversion process to finish
    }
    return ADC1BUF0; // read the buffer with the result
} // readADC
#endif
#elif defined MX3
/* ************************************************************************** */
/***	adc_Init()
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function initializes the analog to digital converter module 
**      in manual sampling mode.
**          
*/
void adc_init(void){

	AD1CON1	=	0; 
    AD1CON1bits.SSRC = 7;   // Internal counter ends sampling and starts conversion (auto convert)
    AD1CON1bits.FORM = 0;   // Integer 16-bit
	// Setup for manual sampling
	AD1CSSL	=	0;
	AD1CON3	=	0x0002;     // ADC Conversion Clock Select bits: TAD = 6 TPB
	AD1CON2	=	0;
    AD1CON2bits.VCFG = 0;   // Voltage Reference Configuration bits: VREFH = AVDD and VREFL = AVSS
	// Turn on ADC
    AD1CON1bits.ON = 1;
} 

/* ************************************************************************** */
/***	adc_read
**
**	Parameters:
**		unsigned char analogPIN - the number of the analog pin that must be read
**
**	Return Value:
**		- the 16 LSB bits contain the result of analog to digital conversion of the analog value of the specified pin
**
**	Description:
**		This function returns the digital value corresponding to the analog pin, 
**      as the result of analog to digital conversion performed by the ADC module. 
**          
*/
unsigned int adc_read(int analogPIN)
{
    int adc_val = 0;
    
    IEC0bits.T2IE = 0;
    AD1CHS = analogPIN << 16;       // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;           // Begin sampling
    while( AD1CON1bits.SAMP );      // wait until acquisition is done
    while( ! AD1CON1bits.DONE );    // wait until conversion is done
 
    adc_val = ADC1BUF0;
    IEC0bits.T2IE = 1;
    return adc_val;
}
#elif defined MICROSTICK_II

/* ************************************************************************** */
/***	adc_init()
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function initializes the analog to digital converter module 
**      in manual sampling mode.
**          
*/
void adc_init(void){
    //Configures the ADC_AN4 pin as analog input.
    TRISBbits.TRISB2 =1; // set AN4 (RB2) as analog input pin 
    ANSELBbits.ANSB2 =1; // enable analog (set pins as analog)
    
	AD1CON1	=	0; 
    AD1CON1bits.SSRC = 7;   // Internal counter ends sampling and starts conversion (auto convert)
    AD1CON1bits.FORM = 0;   // Integer 16-bit
	// Setup for manual sampling
	AD1CSSL	=	0;
	AD1CON3	=	0x0002;     // ADC Conversion Clock Select bits: TAD = 6 TPB
	AD1CON2	=	0;
    AD1CON2bits.VCFG = 0;   // Voltage Reference Configuration bits: VREFH = AVDD and VREFL = AVSS
	// Turn on ADC
    AD1CON1bits.ON = 1;
} 

/* ************************************************************************** */
/***	adc_read
**
**	Parameters:
**		unsigned char analogPIN - the number of the analog pin that must be read
**
**	Return Value:
**		- the 16 LSB bits contain the result of analog to digital conversion of the analog value of the specified pin
**
**	Description:
**		This function returns the digital value corresponding to the analog pin, 
**      as the result of analog to digital conversion performed by the ADC module. 
**          
*/
unsigned int adc_read(int analogPIN)
{
    int adc_val = 0;
    
    //IEC0bits.T2IE = 0;
    AD1CHS = analogPIN << 16;       // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;           // Begin sampling
    while( AD1CON1bits.SAMP );      // wait until acquisition is done
    while( ! AD1CON1bits.DONE );    // wait until conversion is done
 
    adc_val = ADC1BUF0;
    //IEC0bits.T2IE = 1;
    return adc_val;
}

#endif
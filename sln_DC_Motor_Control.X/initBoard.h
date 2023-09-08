/**
  
*/

#ifndef INITBOARD_H
#define INITBOARD_H

#ifdef	__cplusplus
extern "C" {
#endif
/**
    Section: Includes
*/
#include <stdint.h>
#include <xc.h>
/**
    Section: Macros
*/
#define io_init     	ios_init 
#define initIOs         ios_init     	
/**
    Section: Function Prototypes
*/
//void initTimer2( void);
void ios_init(void);

#ifdef	__cplusplus
}
#endif
#endif

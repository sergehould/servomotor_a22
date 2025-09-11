/***********************************************************************************************
 *
 * Combined LCD and UART Drivers for PIC32.
 *
 ***********************************************************************************************
 * FileName:  console.c      
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author   Date            v       Comments on this revision
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * SH		11 Janv. 2020   v1.0    Modify console.c for PIC32MX795F512L
 *                                  Macros conflicts: _UARTx becomes C_UARTx and _LCD becomes C_LCD
 *                                  Must set up the simulator properly: properties->simulator->Option categories-> Uartx Option->  check Enable Uartx IO
 *                                  Provide C++ Compatibility in console32.h: #ifdef __cplusplus extern "C" #endif
  * SH      12 Jan 2021     v1.1    Test Uart2 ISR modify initUart2_wInt() 
 *  SH      8  Feb 2021     v1.2    Got rid of pmp.h.  Static library still needed for  INTEnableSystemMultiVectoredInt()
 *	SH      10  Feb 2021    v1.3	Replace INTEnableSystemMultiVectoredInt() by INTCONbits.MVEC=1; No more library needed
 *          8 March 2021    v1.4	Add void initLCD(void);
 *          13 Sept 2021    v2.0	Add MX3 board compatibility.  Macro MX3 is defined 
 *                                  in properties->xc32-gcc->Processing and messages  
 *                          v2.1    Add call back function _mon_putc ()
 * SH		28 Dec. 2021			The file was renamed console.c.  From now on, it supersedes console32.c. 
 * SH		15 Feb. 2022	v2.3	Add MICROSTICK_II pre-compile conditions to disable UART4 and LCD	
 * SH		21 Feb. 2022	v2.4	Add printUart2FromISR()
 * SH		28 Feb. 2022	v2.4	Add void Uart2_init( void) alias
 * SH		28 Feb. 2022	v2.5    Add Uart1 and Uart2 to fprintf2 because needed
 *                                  for vending machine. 
 * SH		4 March 2022    v2.6    Add mutex and stdio_lock(), stdio_unlock(). A macro RTOS must be define inside MPLABX IDE
 * SH		30 May 2022		v2.7	Add a delay at the end of LCDInitLCDInit()
 * SH       10 Jan. 2023    v2.8    Rename a functions using snake style.
 * SH       21 Feb. 2023    v2.9    Modify initUart2() so it can also receive without interrupt - see also getch2()
 * SH		2 June 2023     v2.10	Add getch_nb(){ and getch_b() functions
 * SH       6 June 2023     v2.11   Add UART3 to_mon_putc()
 *                                  Add uart3_init() 
 * SH       28 08 2023      v2.12   Add send_int16(int16_t data) that sends 2 bytes along with start bye,
 *                                  stop byte and checksum
 * SH       1 Sept. 2023    v2.13   Add send_two_int32() function and rename send_int16() 
 *                                  into send_one_int16().
 * SH       15 Dec. 2023    v2.14   Add a baud rate parameter to the following functions:
 *                                  void uart2_init( int baud);
 *                                  void uart1_init( int baud);
 *                                  void uart3_init( int baud);
 *                                  void uart2_wInt_init(int baud);
 *          
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include <xc.h>
#include "console.h"
#include <sys/attribs.h>
#include <string.h>
//#include "../../projects/DC_Motor_Ctl/sln_DC_Motor_Control.X/configuration.h"

//extern int pos_int32 __attribute__((persistent));
#ifdef RTOS
/* Kernel includes. */
#include "../../Source/include/FreeRTOS.h"
#include "../../Source/include/task.h"
#include "../../Source/include/queue.h" 
#include "../../Source/include/semphr.h"
#include "../../Source/include/croutine.h"
#endif


/*Global variable section */
static int stdio = C_UART2;
/******* LCD section *****************/


#if defined MX3

#define cmdLcdFcnInit       0x3C        // function set command, (8-bit interface, 2 lines, and 5x8 dots)
#define cmdLcdCtlInit       0x0C        // display control set command
#define cmdLcdEntryMode     0x06        // Entry Mode Set
#define cmdLcdClear         0x01		// clear display command
#define cmdLcdRetHome       0x02		// return home command
#define cmdLcdDisplayShift  0x18		// shift display command
#define cmdLcdCursorShift   0x10		// shift cursor command
#define cmdLcdSetDdramPos	0x80	// set DDRAM position command
#define cmdLcdSetCgramPos	0x40	// set CGRAM position command

#define mskBStatus                  0x80             // bit busy 
#define mskShiftRL                  0x04             // bit for direction 
#define displaySetOptionDisplayOn	0x4 // Set Display On option
#define	displaySetOptionCursorOn 	0x2 // Set Cursor On option
#define	displaySetBlinkOn 			0x1 // Set Blink On option


#define posCgramChar0 0		// position in CGRAM for character 0
#define posCgramChar1 8		// position in CGRAM for character 1
#define posCgramChar2 16	// position in CGRAM for character 2
#define posCgramChar3 24	// position in CGRAM for character 3
#define posCgramChar4 32	// position in CGRAM for character 4
#define posCgramChar5 40	// position in CGRAM for character 5
#define posCgramChar6 48	// position in CGRAM for character 6
#define posCgramChar7 56	// position in CGRAM for character 7

#endif
#if defined EXPLORER_16_32
unsigned long _uLCDloops;
static void Wait(unsigned int B);
static void pmp_Init(void);
#endif

#ifndef MICROSTICK_II
static void LCDHome(void);
static void LCDL1Home(void);
static void LCDL2Home(void);
static void LCDClear(void);
#endif      

#if defined EXPLORER_16_32

//static void pmp_Init(void)
//{
//	unsigned int mode,control1,port,addrs,interrupt1;
//	
//    control1 = PMP_ON | PMP_READ_WRITE_EN  | PMP_WRITE_POL_HI | PMP_READ_POL_HI;
//	//mode = BIT_MODE_MASTER_1 | BIT_WAITB_4_TCY  | BIT_WAITM_15_TCY | BIT_WAITE_4_TCY ;
//    mode = PMP_MODE_MASTER1 | PMP_WAIT_BEG_4 | PMP_WAIT_MID_15 | PMP_WAIT_END_4  ;
//	//port = BIT_P0;
//    port = PMP_PEN_0;
//	addrs = 0x0000;
//	interrupt1 = 0x0000;
//	
//	//PMPClose();
//    mPMPClose() ;
//	//PMPOpen(control,mode,port,addrs,interrupt);
//    mPMPOpen(control1,mode,port,interrupt1);
//}	

static void pmp_Init(void)
{
	__builtin_disable_interrupts(); // disable interrupts, remember initial state
    IEC1bits.PMPIE = 0; // disable PMP interrupts
    PMCON = 0; // clear PMCON, like it is on reset
    PMCONbits.PTWREN = 1; // PMENB strobe enabled
    PMCONbits.PTRDEN = 1; // PMRD/PMWR enabled
    PMCONbits.WRSP = 1; // Read/write strobe is active high
    PMCONbits.RDSP = 1; // Read/write strobe is active high
    PMMODE = 0; // clear PMMODE like it is on reset
    PMMODEbits.MODE = 0x3; // set master mode 1, which uses a single strobe
    // Set up wait states. The LCD requires data to be held on its lines
    // for a minimum amount of time.
    // All wait states are given in peripheral bus clock
    // (PBCLK) cycles. PBCLK of 80 MHz in our case
    // so one cycle is 1/80 MHz = 12.5 ns.
    // The timing controls asserting/clearing PMENB (RD4) which
    // is connected to the E pin of the LCD (we refer to the signal as E here)
    // The times required to wait can be found in the LCD controller?s data sheet.
    // The cycle is started when reading from or writing to the PMDIN SFR.
    // Note that the wait states for writes start with minimum of 1 (except WAITE)
    // We add some extra wait states to make sure we meet the time and
    // account for variations in timing amongst different HD44780 compatible parts.
    // The timing we use here is for the KS066U which is faster than the HD44780.
    PMMODEbits.WAITB = 0x3; // Tas in the LCD datasheet is 60 ns
    PMMODEbits.WAITM = 0xF; // PWeh in the data sheet is 230 ns (we don?t quite meet this)
    // If not working for your LCD you may need to reduce PBCLK
    PMMODEbits.WAITE = 0x1; // after E is low wait Tah (10ns)
    PMAEN |= 1 << 0; // PMA is an address line
    PMCONbits.ON = 1; // enable the PMP peripheral
    __builtin_enable_interrupts();
}	
#endif

/***************************************************************
Name:	void LCDInit(void)
Description: Initializes the LCD using pmp bus.

****************************************************************/
//void LCDInit(void)
//{	
//    pmp_Init();
//
//	_uLCDloops = LCD_STARTUP;
//	Wait(_uLCDloops);
//
//	_uLCDloops = LCD_F_INSTR;
//	//PMDIN1 = 0b00111000;			// Set the default function
//    PMDIN = 0b00111000;			// Set the default function
//	Wait(_uLCDloops);
//
//	_uLCDloops = LCD_STARTUP;
//	Wait(_uLCDloops);
//
//	_uLCDloops = LCD_F_INSTR;
//	PMDIN = 0b00001100;
//	Wait(_uLCDloops);
//
//	_uLCDloops = LCD_STARTUP;
//	Wait(_uLCDloops);
//
//	_uLCDloops = LCD_S_INSTR;
//	PMDIN = 0b00000001;			// Clear the display
//	Wait(_uLCDloops);
//
//	_uLCDloops = LCD_STARTUP;
//	Wait(_uLCDloops);
//
//	_uLCDloops = LCD_S_INSTR;
//	PMDIN = 0b00000110;			// Set the entry mode
//     
//	Wait(_uLCDloops);
//
//	LCDClear();
//	LCDHome();
//}


#if defined EXPLORER_16_32
/* lcd_init */
void lcd_init(void){	
    pmp_Init();

	_uLCDloops = LCD_STARTUP;
	Wait(_uLCDloops);

	_uLCDloops = LCD_F_INSTR;
	//PMDIN1 = 0b00111000;			// Set the default function
    PMDIN = 0b00111000;			// Set the default function
	Wait(_uLCDloops);

	_uLCDloops = LCD_STARTUP;
	Wait(_uLCDloops);

	_uLCDloops = LCD_F_INSTR;
	PMDIN = 0b00001100;
	Wait(_uLCDloops);

	_uLCDloops = LCD_STARTUP;
	Wait(_uLCDloops);

	_uLCDloops = LCD_S_INSTR;
	PMDIN = 0b00000001;			// Clear the display
	Wait(_uLCDloops);

	_uLCDloops = LCD_STARTUP;
	Wait(_uLCDloops);

	_uLCDloops = LCD_S_INSTR;
	PMDIN = 0b00000110;			// Set the entry mode
     
	Wait(_uLCDloops);

	LCDClear();
	LCDHome();
	_uLCDloops = LCD_SETTLE_TIME;
    Wait(_uLCDloops);
}
#elif defined MX3
#define LCD_SetWriteDdramPosition(bAddr) LCD_WriteCommand(cmdLcdSetDdramPos | bAddr);

/* ------------------------------------------------------------ */
/***    Delay10Us
**
**	Synopsis:
**      Delay10Us(100)  // 100*10uS = 1mS
**
**	Parameters:
**		t10usDelay - the amount of time you wish to delay in tens of microseconds
**
**	Return Values:
**      none
**
**	Errors:
**		none
**
**	Description:
**		This procedure delays program execution for the specified number
**      of microseconds. This delay is not precise.
**		
**	Note:
**		This routine is written with the assumption that the
**		system clock is 80 MHz.
*/
void DelayAprox10Us( unsigned int  t100usDelay )
{
    int j;
    while ( 0 < t100usDelay )
    {
        t100usDelay--;
        j = 14;
        while ( 0 < j )
        {
            j--;
        }   // end while 
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing
         
    }   // end while
}

/* ------------------------------------------------------------ */
/***	LCD_ConfigurePins
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function configures the digital pins involved in the LCD module: 
**      The following digital pins are configured as digital outputs: LCD_DISP_RS, LCD_DISP_RW, LCD_DISP_EN
**      The following digital pins are configured as digital inputs: LCD_DISP_RS.
**      The function uses pin related definitions from config.h file.
**      This is a low-level function called by LCD_Init(), so user should avoid calling it directly.
**      
**          
*/
#define tris_LCD_DISP_RS    TRISBbits.TRISB15
#define tris_LCD_DISP_RW    TRISDbits.TRISD5
#define tris_LCD_DISP_EN    TRISDbits.TRISD4
#define ansel_LCD_DISP_RS   ANSELBbits.ANSB15
#define rp_LCD_DISP_RS      RPB15R
#define rp_LCD_DISP_RW      RPD5R
#define rp_LCD_DISP_EN      RPD4R
#define ansel_LCD_DB2        ANSELEbits.ANSE2
#define ansel_LCD_DB4        ANSELEbits.ANSE4
#define ansel_LCD_DB5        ANSELEbits.ANSE5
#define ansel_LCD_DB6        ANSELEbits.ANSE6
#define ansel_LCD_DB7        ANSELEbits.ANSE7

void LCD_ConfigurePins()
{
    // set control pins as digital outputs.
    tris_LCD_DISP_RS = 0;
    tris_LCD_DISP_RW = 0;
    tris_LCD_DISP_EN = 0;
    
    // disable analog (set pins as digital))
    ansel_LCD_DISP_RS = 0;
    
    // default (IO) function for remapable pins
    rp_LCD_DISP_RS = 0;
    rp_LCD_DISP_RW = 0;
    rp_LCD_DISP_EN = 0;
    
    // make data pins digital (disable analog)
    ansel_LCD_DB2 = 0;
    ansel_LCD_DB4 = 0;
    ansel_LCD_DB5 = 0;
    ansel_LCD_DB6 = 0;
    ansel_LCD_DB7 = 0;
}

/* ------------------------------------------------------------ */
/***	LCD_WriteByte
**
**	Parameters:
**		unsigned char bData - the data to be written to LCD, over the parallel interface
**
**	Return Value:
**		
**
**	Description:
**		This function writes a byte to the LCD. 
**      It implements the parallel write using LCD_DISP_RS, LCD_DISP_RW, LCD_DISP_EN, 
**      LCD_DISP_RS pins, and data pins. 
**      For a better performance, the data pins are accessed using a pointer to 
**      the register byte where they are allocated.
**      This is a low-level function called by LCD write functions, so user should avoid calling it directly.
**      The function uses pin related definitions from config.h file.
**      
**          
*/

#define tris_LCD_DATA       TRISE
#define msk_LCD_DATA        0xFF
#define  lat_LCD_DISP_EN    LATDbits.LATD4
#define  lat_LCD_DISP_RW    LATDbits.LATD5
void LCD_WriteByte(unsigned char bData)
{
    DelayAprox10Us(5);  
	// Configure IO Port data pins as output.
    tris_LCD_DATA &= ~msk_LCD_DATA;
    DelayAprox10Us(5);  
	// clear RW
	lat_LCD_DISP_RW = 0;

    // access data as contiguous 8 bits, using pointer to the LSB byte of LATE register
    unsigned char *pLCDData = (unsigned char *)(0xBF886430);
    *pLCDData = bData;

    DelayAprox10Us(10);   

	// Set En
	lat_LCD_DISP_EN = 1;    

    DelayAprox10Us(5);
	// Clear En
	lat_LCD_DISP_EN = 0;

    DelayAprox10Us(5);
	// Set RW
	lat_LCD_DISP_RW = 1;
}

/* ------------------------------------------------------------ */
/***	LCD_WriteCommand
**
**	Parameters:
**		unsigned char bCmd -  the command code byte to be written to LCD
**
**	Return Value:
**		
**
**	Description:
**		Writes the specified byte as command. 
**      It clears the RS and writes the byte to LCD. 
**      The function uses pin related definitions from config.h file.
**      
**          
*/

#define lat_LCD_DISP_RS     LATBbits.LATB15
void LCD_WriteCommand(unsigned char bCmd)
{ 
	// Clear RS
	lat_LCD_DISP_RS = 0;

	// Write command byte
	LCD_WriteByte(bCmd);
}

/* ------------------------------------------------------------ */
/***	LCD_WriteDataByte
**
**	Parameters:
**		unsigned char bData -  the data byte to be written to LCD
**
**	Return Value:
**		
**
**	Description:
**      Writes the specified byte as data. 
**      It sets the RS and writes the byte to LCD. 
**      The function uses pin related definitions from config.h file.
**      This is a low-level function called by LCD write functions, so user should avoid calling it directly.
**      
**          
*/
void LCD_WriteDataByte(unsigned char bData)
{
	// Set RS 
	lat_LCD_DISP_RS = 1;

	// Write data byte
	LCD_WriteByte(bData);
}

/* ------------------------------------------------------------ */
/***	LCD_DisplaySet
**
**  Synopsis:
**				LCD_DisplaySet(displaySetOptionDisplayOn | displaySetOptionCursorOn);
**
**	Parameters:
**		unsigned char bDisplaySetOptions -  display options
**					Possible options (to be OR-ed)
**						displaySetOptionDisplayOn - display ON
**						displaySetOptionCursorOn - cursor ON
**						displaySetBlinkOn - cursor blink ON
**
**	Return Value:
**		
**
**	Description:
**      The LCD is initialized according to the parameter bDisplaySetOptions. 
**      If one of the above mentioned optios is not OR-ed, 
**      it means that the OFF action is performed for it.
**      
**          
*/
void LCD_DisplaySet(unsigned char bDisplaySetOptions)
{
	LCD_WriteCommand(cmdLcdCtlInit | bDisplaySetOptions);
}


/* ------------------------------------------------------------ */
/***	LCD_DisplayClear
**
**	Parameters:
**
**	Return Value:
**		
**	Description:
**      Clears the display and returns the cursor home (upper left corner, position 0 on row 0). 
**      
**          
*/
void LCD_DisplayClear()
{
	LCD_WriteCommand(cmdLcdClear);
}

/* ------------------------------------------------------------ */
/***	LCD_InitSequence
**
**  Synopsis:
**              LCD_InitSequence(displaySetOptionDisplayOn);//set the display on
**
**	Parameters:
**		unsigned char bDisplaySetOptions -  display options
**					Possible options (to be OR-ed)
**						displaySetOptionDisplayOn - display ON
**						displaySetOptionCursorOn - cursor ON
**						displaySetBlinkOn - cursor blink ON
**
**	Return Value:
**		
**
**	Description:
**		This function performs the initializing (startup) sequence. 
**      The LCD is initialized according to the parameter bDisplaySetOptions. 
**      
**          
*/
void LCD_InitSequence(unsigned char bDisplaySetOptions)
{
	//	wait 40 ms
	DelayAprox10Us(40000);  
	// Function Set
	LCD_WriteCommand(cmdLcdFcnInit);
	// Wait ~100 us
	DelayAprox10Us(10);
	// Function Set
	LCD_WriteCommand(cmdLcdFcnInit);
	// Wait ~100 us
	DelayAprox10Us(10);	// Display Set
	LCD_DisplaySet(bDisplaySetOptions);
	// Wait ~100 us
	DelayAprox10Us(10);
	// Display Clear
	LCD_DisplayClear();
	// Wait 1.52 ms
	DelayAprox10Us(160);
    // Entry mode set
	LCD_WriteCommand(cmdLcdEntryMode);
    	// Wait 1.52 ms
	DelayAprox10Us(160);
}


void lcd_init(void){	
        LCD_ConfigurePins();
        LCD_InitSequence(displaySetOptionDisplayOn);
}


/* ------------------------------------------------------------ */
/***	LCD_WriteStringAtPos
**
**  Synopsis:
**      LCD_WriteStringAtPos("Demo", 0, 0);
**
**	Parameters:
**      char *szLn	- string to be written to LCD
**		int idxLine	- line where the string will be displayed
**          0 - first line of LCD
**          1 - second line of LCD
**		unsigned char idxPos - the starting position of the string within the line. 
**                                  The value must be between:
**                                      0 - first position from left
**                                      39 - last position for DDRAM for one line
**                                  
**
**	Return Value:
**		
**	Description:
**		Displays the specified string at the specified position on the specified line. 
**		It sets the corresponding write position and then writes data bytes when the device is ready.
**      Strings longer than 40 characters are trimmed. 
**      It is possible that not all the characters will be visualized, as the display only visualizes 16 characters for one line.
**      
**          
*/
void LCD_WriteStringAtPos(char *szLn, unsigned char idxLine, unsigned char idxPos)
{
	// crop string to 0x27 chars
	int len = strlen(szLn);
	if(len > 0x27)
	{
        szLn[0x27] = 0; // trim the string so it contains 40 characters 
		len = 0x27;
	}

	// Set write position
	unsigned char bAddrOffset = (idxLine == 0 ? 0: 0x40) + idxPos;
	LCD_SetWriteDdramPosition(bAddrOffset);

	unsigned char bIdx = 0;
	while(bIdx < len)
	{
		LCD_WriteDataByte(szLn[bIdx]);
		bIdx++;
	}
}

#endif

/***************************************************************
Name:	void LCDHome(void)
Description: Sets the position home.

****************************************************************/
#if defined EXPLORER_16_32
static void LCDHome(void)
{
	_uLCDloops = LCD_S_INSTR;
	PMADDR = 0x0000;
    //PMPSetAddress(0x0000); 
	PMDIN = 0b00000010;
	while(_uLCDloops)
	_uLCDloops--;
}
#elif defined MX3

/* ------------------------------------------------------------ */
/***	LCD_SetWriteCgramPosition
**
**
**	Parameters:
**      unsigned char bAdr	- the write location. The position in CGRAM where the next data write operations will put bytes.
**
**	Return Value:
**		
**	Description:
**		Sets the DDRAM write position. This is the location where the next data write operation will be performed.
**		Writing to a location auto-increments the write location.
**      This is a low-level function called by LCD_WriteBytesAtPosCgram(), so user should avoid calling it directly.
 **      
**          
*/
void LCD_SetWriteCgramPosition(unsigned char bAdr)
{
	unsigned char bCmd = cmdLcdSetCgramPos | bAdr;
	LCD_WriteCommand(bCmd);
}

static void LCDHome(void)
{
    LCD_WriteCommand(cmdLcdRetHome);
}

/* ------------------------------------------------------------ */
/***	LCD_WriteBytesAtPosCgram
**
**  Synopsis:
**      LCD_WriteBytesAtPosCgram(userDefArrow, 8, posCgramChar0);
**
**	Parameters:
**		unsigned char *pBytes	- pointer to the string of bytes
**		unsigned char len		- the number of bytes to be written
**		unsigned char bAdr		- the position in CGRAM where bytes will be written
**
**	Return Value:
**		
**	Description:
**		Writes the specified number of bytes to CGRAM starting at the specified position. 
**      This allows user characters to be defined.
**		It sets the corresponding write position and then writes data bytes when the device is ready.
**      
**          
*/
void LCD_WriteBytesAtPosCgram(unsigned char *pBytes, unsigned char len, unsigned char bAdr)
{
	// Set write position
	LCD_SetWriteCgramPosition(bAdr);

	// Write the string of bytes that define the character to CGRAM
	unsigned char idx = 0;
	while(idx < len)
	{
		LCD_WriteDataByte(pBytes[idx]);
		idx++;
	}
}


#endif
/***************************************************************
Name:	void LCDL1Home(void)
Description: Sets the position home on line1

****************************************************************/
#if defined EXPLORER_16_32
static void LCDL1Home(void)
{
	_uLCDloops = LCD_S_INSTR;
	PMADDR = 0x0000;
    //PMPSetAddress(0x0000); 
	PMDIN = 0b10000000;
	while(_uLCDloops)
	_uLCDloops--;
}
#elif defined MX3
static void LCDL1Home(void){
    LCD_WriteCommand(cmdLcdRetHome);
}

#endif
/***************************************************************
Name:	void LCDL2Home(void)
Description: Sets the position home on line2.

****************************************************************/
#if defined EXPLORER_16_32
static void LCDL2Home(void)
{
	_uLCDloops = LCD_S_INSTR;
	PMADDR = 0x0000;
	//PMPSetAddress(0x0000); 
	PMDIN = 0b11000000;
	while(_uLCDloops)
	_uLCDloops--;
}
#elif defined MX3
static void LCDL2Home(void){
    LCD_SetWriteDdramPosition(0x40);
}
#endif

/***************************************************************
Name:	void LCDClear(void)
Description: Clears the whole LCD

****************************************************************/
#if defined EXPLORER_16_32
static void LCDClear(void)
{
	_uLCDloops = LCD_S_INSTR;
	PMADDR = 0x0000;
	//PMPSetAddress(0x0000); 
	PMDIN = 0b00000001;
	while(_uLCDloops)
	_uLCDloops--;
}
#endif
/***************************************************************
Name:	void LCDPut(char A)
Description: Put a character at the current position.

****************************************************************/
#if defined EXPLORER_16_32
void LCDPut(char A)
{
	_uLCDloops = LCD_F_INSTR;
	PMADDR = 0x0001;
    //PMPSetAddress(0x0001); 
	PMDIN = A;
	while(_uLCDloops)
	_uLCDloops--;
	Nop();
	Nop();
	Nop();
	Nop();
}
#elif defined MX3
void LCDPut(char A)
{
    LCD_WriteDataByte(A);
}

#endif

/***************************************************************
Name:	void Wait(unsigned int B)
Description: Short delay

****************************************************************/
#if defined EXPLORER_16_32
void Wait(unsigned int B)
{
	while(B)
	B--;
}
#endif

#ifndef MICROSTICK_II
/***************************************************************
Name:	void DisplayMSG( char *array)
Description: Dump a string to the current position. If it 
				reaches the end of line1, it will continue on
				line2.

****************************************************************/
void DisplayMSG( char *array)
{
  unsigned char i=0,line=1;	
	LCDL1Home();	
	 while (*array)           // Continue display characters from STRING untill NULL character appears.
	 {
	  LCDPut(*array++);  // Display selected character from the STRING.
	  if (i>19 && line==1)
	  {
	   LCDL2Home();
	   line++;
	  }
	   i++;	        
     }
}

/***************************************************************
Name:	void LCDPutString( char *array)
Description: Dump a string to the current position. Does not take 
				into account the end of the line.

****************************************************************/
void LCDPutString( char *array)
{
	 while (*array)           // Continue display characters from STRING untill NULL character appears.
	 {
	  LCDPut(*array++);  // Display selected character from the STRING.
     }
}

#endif

/***************************************************************
Name:	void LCDPos2(unsigned char row)
Description: Position the cursor to a specific position on line2.

****************************************************************/
#if defined EXPLORER_16_32
void LCDPos2(unsigned char row)
{
    unsigned char temp;
	_uLCDloops = LCD_S_INSTR;
	PMADDR = 0x0000;
    //PMPSetAddress(0x0000); 
  //  PMDIN = 0b11001010 ;
    temp = 0b11000000 | row;
	PMDIN = temp;
	while(_uLCDloops)
	_uLCDloops--;
}
#elif defined MX3
void LCDPos2(unsigned char idxPos)
{
    unsigned char bAddrOffset = 0x40 + idxPos;
	LCD_SetWriteDdramPosition(bAddrOffset);
}
#endif
/***************************************************************
Name:	void LCDPos1(unsigned char row)
Description: Position the cursor to a specific position on line1.

****************************************************************/
#if defined EXPLORER_16_32
void LCDPos1(unsigned char row)
{
    unsigned char temp;
	_uLCDloops = LCD_S_INSTR;
	PMADDR = 0x0000;
    //PMPSetAddress(0x0000); 
  //  PMDIN = 0b11001010 ;
    temp = 0b10000000 | row;
	PMDIN = temp;
	while(_uLCDloops)
	_uLCDloops--;
}
#elif defined MX3

void LCDPos1(unsigned char idxPos)
{
    unsigned char bAddrOffset = idxPos;
	LCD_SetWriteDdramPosition(bAddrOffset);
}


/***************************************************************
Name:	void LCDPos(char pos)
Description: Position the cursor to a specific position.
The position can be anywhere between 0 and 31.
Line1: position 0 to 15
Line2: position 16 to 31

****************************************************************/
void LCDPos(char pos)
{
    //Delay100TCYx(1);//Wait for some time to let the LCD process it
      if(pos<16){
            LCD_SetWriteDdramPosition(pos);
      } 
      else  LCD_SetWriteDdramPosition(pos-16+0x40);
}
#endif

/*********** Uart2 section **************************************************/
#if defined EXPLORER_16_32
//
//void UART2_Initialize(void)
//{
///**    
//     Set the UART2 module to the options selected in the user interface.
//     Make sure to set LAT bit corresponding to TxPin as high before UART initialization
//*/
//    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
//    U2MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
//    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
//    U2STA = 0x0000;
//    // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
//    U2BRG = 0x01A0;
//    
//    U2MODEbits.UARTEN = 1;  // enabling UARTEN bit
//    U2STAbits.UTXEN = 1; 
//   
//}

//
// I/O definitions for the Explorer16 using hardware flow control
#define CTS    	_RF12              // Cleart To Send, input, HW handshake
#define RTS     _RF13               // Request To Send, output, HW handshake
#define TRTS    TRISFbits.TRISF13   // tris control for RTS pin

/* U2BRG (BRATE)
U2BRG = (PBCLK  / 16 / baudrate) -1 ; for BREGH=0
*/
// timing and baud rate calculations
//baud 115200
#define BRATE   21        // (40000000/16/115200)-1
// baud 19200 if fcy = 5MHz
//#define BRATE   15        // (5000000/16/19200)-1
//baud 9600
//#define BRATE   	259        // (40000000/16/9600)-1
//#define U_ENABLE 	0x8008      // enable the UART peripheral (BREGH=1)
#define U_ENABLE 	0x8000      // 
#define U_TX    	0x0400      // enable transmission
#define U_TX_RX     0x1400; // enable TX and RX
   
/**********************************
 Initialize the UART2 serial port
**********************************/
void uart2_init( int baud){
   U2BRG    =(40000000/16/baud)-1;   
   U2MODE    = U_ENABLE;
   U2STA    = U_TX_RX;
   //U2STA    = U_TX;
   //TRTS    = 0;        // make RTS output
   //RTS     = 1;        // set RTS default status
} // uart2_init

void Uart2_init( void)
{
   U2BRG    = BRATE;    
   U2MODE    = U_ENABLE;
   U2STA    = U_TX_RX;
   //U2STA    = U_TX;
   //TRTS    = 0;        // make RTS output
   //RTS     = 1;        // set RTS default status

} // uart2_init


/**********************************
 initialize the UART2 serial port 
 with interrupt.  
 See ISR at the end of this document
 **********************************/
void uart2_wInt_init( int baud)
{
   U2BRG     =(40000000/16/baud)-1;   
   U2MODE    = U_ENABLE ;     // enable the UART peripheral (BREGH=1)
  // U2MODE = 0x8000;
   //U2STA    = U_TX;      // enable transmission
   //IFS1bits.U2RXIF=0;  
   //_U2RXIP=1;  // Interrutp priority 1
   //IEC1bits.U2RXIE=1;  // if interrupt driven RX only
   INTCONbits.MVEC=1; // enable multiVectoredInt
   __builtin_disable_interrupts();
    U2STAbits.URXISEL=0;//interupt when 1 bytes in buffer
    U2STAbits.UTXISEL0=1;//interrupt when last byte transmitted
    U2STAbits.UTXEN = 1;//enable
   // mU2EIntEnable(1);
    U2STAbits.URXEN = 1;
    IPC8bits.U2IP=1;
	IFS1bits.U2RXIF=0; //clears flag
    //mU2RXIntEnable(1);
    U2STAbits.URXEN = 1;
    IEC1bits.U2RXIE = 1; // enable the RX interrupt
    U2MODEbits.ON = 1;
	//INTEnableSystemMultiVectoredInt();
    __builtin_enable_interrupts();
} // initUart2_wInt



/****************************************
Send a singe character to the UART2 
serial port.

input: 
	Parameters:
		char c 	character to be sent
output:
	return:
		int		return the character sent.
*****************************************/
int putc2(char c)
{
  // while ( CTS);              // wait for !CTS, clear to send
   while ( U2STAbits.UTXBF);   // wait while Tx buffer full
   U2TXREG = c;
   return c;
} 

/****************************************
Same as putc2() but  w/o hardware control
*****************************************/
int putc2_noHard(char c)
{
   while ( U2STAbits.UTXBF);   // wait while Tx buffer is still full
   U2TXREG = c;
   return c;
}

/******************************************************************************
 * Blocks waiting for a new character to arrive to the UART2 serial port
 * Returns the character only if not empty
******************************************************************************/
char getch_b( void)
{
    //RTS = 0;            // assert Request To Send !RTS
   while ( !U2STAbits.URXDA){ // wait for a new character to arrive
#ifdef RTOS
       vTaskDelay(10/portTICK_RATE_MS); // slack time when using RTOS
#endif
   }
   return U2RXREG;      // read the character from the receive buffer
   //RTS = 1;
}// 


/****************************************
 * Non-blocking polling for a new character to arrive to the UART2 serial port
 * Returns the character if not empty
 * Returns 0xff if empty
*****************************************/
//char get_byte( void){
//    //RTS = 0;            // assert Request To Send !RTS
//   if( !U2STAbits.URXDA){
//       return 0xff;      // read the character from the receive buffer
//   }
//   else {
//       return U2RXREG; // if not empty
//   }
//   //RTS = 1;
//}// 

/*
 Non-blocking polling function for a new character to be
 Received from the UART2 serial port.
 Returns the received character if not empty.
 Returns the previous value if empty.
*/
#ifndef SIMULATION
char get_byte( int p){
   static char last =0;
   if(!(U2STAbits.URXDA == 1)){
       return last;      
   }
   else {
       last = U2RXREG; // if not empty
       return last;
   }
}// 
#endif


   /*******************************************************************************
   Function: puts2( char *str )

   Precondition:
      initUart must be called prior to calling this routine.

   Overview:
      This function prints a string of characters to the UART.

   Input: Pointer to a null terminated character string.

   Output: None.

   *******************************************************************************/
   void puts2( char *str )
   {
      unsigned char c;

      while( (c = *str++) )
         putc2(c);
   }

   
   // to erase because putI8 supercedes it
void outUint8(unsigned char u8_x) {
  unsigned char u8_c;
   putc2('0');
   putc2('X');
  u8_c = (u8_x>>4)& 0xf;
  if (u8_c > 9) putc2('A'+u8_c-10);
  else putc2('0'+u8_c);
  //LSDigit
  u8_c= u8_x & 0xf;
  if (u8_c > 9) putc2('A'+u8_c-10);
  else putc2('0'+u8_c);
}

/****************************************
Precondition:
    initUart2() must be called prior to calling 
	this routine.

	Overview: send an 8 bit integer value to 	
	the UART2 serial port
	Example: putI8(55);
*****************************************/
void putI8(unsigned char u8_x) {
  unsigned char u8_c;
   putc2('0');
   putc2('X');
  u8_c = (u8_x>>4)& 0xf;
  if (u8_c > 9) putc2('A'+u8_c-10);
  else putc2('0'+u8_c);
  //LSDigit
  u8_c= u8_x & 0xf;
  if (u8_c > 9) putc2('A'+u8_c-10);
  else putc2('0'+u8_c);
}

//
//bool UART2_is_tx_ready(void)
//{
//    return (IFS1bits.U2TXIF && U2STAbits.UTXEN);
//}
//
//bool UART2_is_rx_ready(void)
//{
//    return IFS1bits.U2RXIF;
//}
//
//bool UART2_is_tx_done(void)
//{
//    return U2STAbits.TRMT;
//}

uint8_t UART2_Read(void)
{
    while(!(U2STAbits.URXDA == 1))
    {
        
    }

    if ((U2STAbits.OERR == 1))
    {
        U2STAbits.OERR = 0;
    }

    

    return U2RXREG;
}

void UART2_Write(uint8_t txData){
    while(U2STAbits.TRMT == 0);
    U2TXREG =  txData;   
}

//UART2_STATUS UART2_StatusGet (void)
//{
//    return U2STA;
//}

/****************************************
	ISR for Uart2 rx
*****************************************/

//void _ISR_NO_PSV _U2RXInterrupt( void )
//{
//    char cChar;
//
//	IFS1bits.U2RXIF = 0;
//	while( U2STAbits.URXDA )
//	{
//		cChar = U2RXREG;
//	}
//
//}
#ifndef SIMULATION
/* Function that sends one byte to the serial port */
void put_byte(char tx) {
    UART2_Write(tx);
}
/*
 Function that splits an int16 into two bytes
 and then sends them to UART2 serial port using a
 start byte, stop byte and checksum byte.
*/ 

#define START_BYTE 0x7E
#define STOP_BYTE 0x7F

void send_one_int16(int16_t data) {
    uint8_t start_byte = START_BYTE;
    uint8_t stop_byte = STOP_BYTE;
    uint8_t data_bytes[2] = { (data >> 8) & 0xFF, data & 0xFF };
    uint8_t checksum = start_byte + stop_byte + data_bytes[0] + data_bytes[1];
    
    UART2_Write(start_byte);    
    UART2_Write(data_bytes[0]);
    UART2_Write(data_bytes[1]);
    UART2_Write(checksum);
    UART2_Write(stop_byte);
    
    //putchar(start_byte);
//    putchar(data_bytes[0]);
//    putchar(data_bytes[1]);
//    putchar(checksum);
//    putchar(stop_byte);
}

/* Function that splits two int32 into eight bytes
 * and then it sends them using a
 * start byte, stop byte and checksum byte.
 * start byte is 0x03 and
 * stop byte is 0xfc.
*/
void send_two_int32(int sp, int pv){
        char buff2[100], debug=0;
        char *buffer;
        unsigned int i;
        uint8_t sp_0, sp_1, sp_2, sp_3, pv_0, pv_1, pv_2, pv_3;
        sp_0 = sp & 0xff;
        sp_1 = (sp >> 8) & 0xff;
        sp_2 = (sp >> 16) & 0xff;
        sp_3 = (sp >> 24) & 0xff;
        pv_0 = pv & 0xff;
        pv_1 = (pv >> 8) & 0xff;
        pv_2 = (pv >> 16) & 0xff;        
        pv_3 = (pv >> 24) & 0xff;             
        buff2[0]=0x03;  // must be also set in the visualizer
        buff2[1]=sp_0 ; //LSB
        buff2[2]=sp_1 ;    
        buff2[3]=sp_2 ;
        buff2[4]=sp_3 ;    //MSB 
        buff2[5]=pv_0 ; //LSB
        buff2[6]=pv_1 ;    
        buff2[7]=pv_2 ;
        buff2[8]=pv_3 ;    //MSB      
        buff2[9]=0xfc; // must be also set in the visualizer
        //buff2[10]=0;
        
        buffer = &buff2[0];

        while(U2STAbits.TRMT == 0);  
        for (i = 10; i; --i)
        {
                    while(U2STAbits.TRMT == 0);
                    U2TXREG = *(char*)buffer++;        
        }
}

/* Does not receive properly. 
 Receives only the first frame of 5 bytes at reset. 
 The next frames are not detected - it seems that 
 it receives only one of the 5 bytes.
 I tried to flush the U2RXREG to no avail
 I also reduced the reading rate to no avail 
 I replace get_byte() by getch_b() to no avail 
 The receiver seems to have NO buffer */
int16_t rec_one_int16_nb() {
    unsigned int byte;
    unsigned int start_byte = START_BYTE;
    unsigned int stop_byte = STOP_BYTE;
    unsigned int data_bytes[2];
    unsigned char checksum = start_byte + stop_byte;
    int i;
    static unsigned int last =0;
    

    if ((byte =get_byte(1)) != start_byte) {
        return last;
    }

    for (i = 0; i < 2; i++) {
        data_bytes[i] =get_byte(1);
        checksum += data_bytes[i];
    }
    byte =get_byte(1);
    if (byte != checksum) {
        // Checksum error
        return last;
    }
    byte =get_byte(1);
    if (byte != stop_byte) {
        // Stop byte error
        return last;
    }
    last = (data_bytes[1] << 8) | data_bytes[0];
    
//   while(!(U2STAbits.URXDA == 1)){
//      toto =U2RXREG;      // read the character from the receive buffer
//   }
    return last;
}
#endif
/***************** Uart1 section************************************/


void UART1_Initialize(int baud)
{
/**    
     Set the UART1 module to the options selected in the user interface.
     Make sure to set LAT bit corresponding to TxPin as high before UART initialization
*/
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U1STA = 0x00;

    U1BRG    =(40000000/16/baud)-1;   
    U1MODEbits.UARTEN = 1;   // enabling UART ON bit
    U1STAbits.UTXEN = 1;
}

void uart1_init(int baud){
    UART1_Initialize(baud);
}

uint8_t UART1_Read(void)
{
    while(!(U1STAbits.URXDA == 1))
    {
        
    }

    if ((U1STAbits.OERR == 1))
    {
        U1STAbits.OERR = 0;
    }
    
    return U1RXREG;
}

void UART1_Write(uint8_t txData)
{
    while(U1STAbits.UTXBF == 1)
    {
        
    }

    U1TXREG = txData;    // Write the data byte to the USART.
}

//bool UART1_IsRxReady(void)
//{
//    return U1STAbits.URXDA;
//}
//
//bool UART1_IsTxReady(void)
//{
//    return (U1STAbits.TRMT && U1STAbits.UTXEN );
//}
//
//bool UART1_IsTxDone(void)
//{
//    return U1STAbits.TRMT;
//}

//int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) 
//{
//    unsigned int i;
//
//    for (i = len; i; --i)
//    {
//        UART1_Write(*(char*)buffer++);
//    }
//    return(len);
//}

/*******************************************************************************

  !!! Deprecated API !!!
  !!! These functions will not be supported in future releases !!!

*******************************************************************************/

//uint16_t __attribute__((deprecated)) UART1_StatusGet (void)
//{
//    return U1STA;
//}
//
//void __attribute__((deprecated)) UART1_Enable(void)
//{
//    U1MODEbits.UARTEN = 1;
//    U1STAbits.UTXEN = 1;
//}
//
//void __attribute__((deprecated)) UART1_Disable(void)
//{
//    U1MODEbits.UARTEN = 0;
//    U1STAbits.UTXEN = 0;
//}

/**************************UART3 section******************************/
void uart3_init( int baud)
{
    
   U3BRG    =(40000000/16/baud)-1;      
   U3MODE    = U_ENABLE;
   U3STA    = U_TX_RX;
   //U2STA    = U_TX;
   //TRTS    = 0;        // make RTS output
   //RTS     = 1;        // set RTS default status
} // uart2_init


#elif defined MX3

/* Uart4 section */
/***	UART_InitPoll
**
**	Parameters:
**		unsigned int baud - UART baud rate.
**                                     for example 115200 corresponds to 115200 baud			
**
**	Return Value:
**		
**
**	Description:
**		This function initializes the hardware involved in the UART module, in 
**      the UART receive without interrupts (polling method).
**      The UART_TX digital pin is configured as digital output.
**      The UART_RX digital pin is configured as digital input.
**      The UART_TX and UART_RX are mapped over the UART4 interface.
**      The UART4 module of PIC32 is configured to work at the specified baud, no parity and 1 stop bit.
**      
**          
*/
void UART4_InitPoll(unsigned int baud)
{
    UART4_ConfigurePins();
    UART4_ConfigureUart(baud);
}

void uart4_init(int baud)
{
    UART4_ConfigurePins();
    UART4_ConfigureUart(baud);
}

/***	UART_ConfigureUart
**
**	Parameters:
**		unsigned int baud - UART baud rate.
**                                     for example 115200 corresponds to 115200 baud
**
**	Return Value:
**		
**
**	Description:
**		This function configures the UART4 hardware interface of PIC32, according 
**      to the provided baud rate, no parity and 1 stop bit, with no interrupts.
**      In order to compute the baud rate value, it uses the peripheral bus frequency definition (PB_FRQ, located in config.h)
**      This is a low-level function called by initialization functions, so user should avoid calling it directly.   
**      
**          
*/
#define PB_FRQ  40000000
void UART4_ConfigureUart(unsigned int baud)
{
    U4MODEbits.ON     = 0;
    U4MODEbits.SIDL   = 0;
    U4MODEbits.IREN   = 0; 
    U4MODEbits.RTSMD  = 0;
    U4MODEbits.UEN0   = 0; 
    U4MODEbits.UEN1   = 0;
    U4MODEbits.WAKE   = 0;
    U4MODEbits.LPBACK = 0; 
    U4MODEbits.ABAUD  = 0;
    U4MODEbits.RXINV  = 0; 
    U4MODEbits.PDSEL1 = 0; 
    U4MODEbits.PDSEL0 = 0; 
    U4MODEbits.STSEL  = 0;  

    
    U4MODEbits.BRGH   = 0; 

    U4BRG = (int)(((float)PB_FRQ/(16*baud) - 1) + 0.5); // add 0.5 just in order to implement the round using the floor approach

    U4STAbits.UTXEN    = 1;
    U4STAbits.URXEN    = 1;
    U4MODEbits.ON      = 1; 
    
}


/***	UART_ConfigurePins
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function configures the digital pins involved in the UART module: 
**      The UART_TX digital pin is configured as digital output.
**      The UART_RX digital pin is configured as digital input.
**      The UART_TX and UART_RX are mapped over the UART4 interface.
**      The function uses pin related definitions from config.h file.
**      This is a low-level function called by UART_Init(), so user should avoid calling it directly.   
**          
*/
void UART4_ConfigurePins()
{
     TRISFbits.TRISF12  = 0;   //TX digital output
    RPF12R = 2;     // 0010 U4TX
    
    TRISFbits.TRISF13 = 1;   //RX digital input
    U4RXR = 9;     // 1001 RF13
}




/***	UART_PutChar
**
**	Parameters:
**          char ch -   the character to be transmitted over UART.
**
**	Return Value:
**		
**
**	Description:
**		This function transmits a character over UART4. 
**      
**          
*/
void UART_PutChar(char ch)
{
    while(U4STAbits.UTXBF == 1);
    U4TXREG = ch;
}

/******************************************************************************
 * Blocks waiting for a new character to arrive to the UART serial port
 * Returns the character only if not empty
******************************************************************************/
char getch_b( void){
   while ( !U4STAbits.URXDA){ // wait for a new character to arrive
#ifdef RTOS
       vTaskDelay(10/portTICK_RATE_MS); // slack time when using RTOS
#endif
   }
   return U4RXREG;      // read the character from the receive buffer
}

/***	UART_PutString
**
**	Parameters:
**          char szData[] -   the zero terminated string containing characters to be transmitted over UART.
**
**	Return Value:
**		
**
**	Description:
**		This function transmits all the characters from a zero terminated string over UART4. The terminator character is not sent.
**      
**          
*/
void UART_PutString(char szData[])
{
    char *pData = szData;
    while(*pData)
    {
        UART_PutChar((*(pData++)));
    }
}
/****************************** Uart3 section *****************************/

/***	UART_InitPoll
**
**	Parameters:
**		unsigned int baud - UART baud rate.
**                                     for example 115200 corresponds to 115200 baud			
**
**	Return Value:
**		
**
**	Description:
**		This function initializes the hardware involved in the UART module, in 
**      the UART receive without interrupts (polling method).
**      The UART_TX digital pin is configured as digital output.
**      The UART_RX digital pin is configured as digital input.
**      The UART_TX and UART_RX are mapped over the UART1 interface.
**      The UART1 module of PIC32 is configured to work at the specified baud, no parity and 1 stop bit.
**      
**          
*/
void UART3_InitPoll(unsigned int baud)
{
    UART3_ConfigurePins();
    UART3_ConfigureUart(baud);
}

void uart3_init(int baud)
{
    UART3_ConfigurePins();
    UART3_ConfigureUart(baud);
}

/***	UART_ConfigureUart
**
**	Parameters:
**		unsigned int baud - UART baud rate.
**                                     for example 115200 corresponds to 115200 baud
**
**	Return Value:
**		
**
**	Description:
**		This function configures the UART4 hardware interface of PIC32, according 
**      to the provided baud rate, no parity and 1 stop bit, with no interrupts.
**      In order to compute the baud rate value, it uses the peripheral bus frequency definition (PB_FRQ, located in config.h)
**      This is a low-level function called by initialization functions, so user should avoid calling it directly.   
**      
**          
*/

void UART3_ConfigureUart(unsigned int baud)
{
    U3MODEbits.ON     = 0;
    U3MODEbits.SIDL   = 0;
    U3MODEbits.IREN   = 0; 
    U3MODEbits.RTSMD  = 0;
    U3MODEbits.UEN0   = 0; 
    U3MODEbits.UEN1   = 0;
    U3MODEbits.WAKE   = 0;
    U3MODEbits.LPBACK = 0; 
    U3MODEbits.ABAUD  = 0;
    U3MODEbits.RXINV  = 0; 
    U3MODEbits.PDSEL1 = 0; 
    U3MODEbits.PDSEL0 = 0; 
    U3MODEbits.STSEL  = 0;  

    
    U3MODEbits.BRGH   = 0; 

    U3BRG = (int)(((float)PB_FRQ/(16*baud) - 1) + 0.5); // add 0.5 just in order to implement the round using the floor approach

    U3STAbits.UTXEN    = 1;
    U3STAbits.URXEN    = 1;
    U3MODEbits.ON      = 1; 
    
}


/***	UART_ConfigurePins
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function configures the digital pins involved in the UART module: 
**      The UART_TX digital pin is configured as digital output.
**      The UART_RX digital pin is configured as digital input.
**      The UART_TX and UART_RX are mapped over the UART4 interface.
**      The function uses pin related definitions from config.h file.
**      This is a low-level function called by UART_Init(), so user should avoid calling it directly.   
**          
*/
void UART3_ConfigurePins()
{
    // TRISFbits.TRISF12  = 0;   //TX digital output
   RPB5R = 1;     // 0001 U3TX RPB5/RB5/AN5  shared with motor driver pin BIN2 
   // disable analog (set pins as digital))
    ANSELBbits.ANSB5 = 0;
    
    /* No input selected yet */
    //TRISFbits.TRISF13 = 1;   //RX digital input
    //U1RXR = 9;     // 1001 
}




/***	UART_PutChar
**
**	Parameters:
**          char ch -   the character to be transmitted over UART.
**
**	Return Value:
**		
**
**	Description:
**		This function transmits a character over UART4. 
**      
**          
*/
void UART_PutChar3(char ch)
{
    while(U3STAbits.UTXBF == 1);
    U3TXREG = ch;
}

/***	UART_PutString
**
**	Parameters:
**          char szData[] -   the zero terminated string containing characters to be transmitted over UART.
**
**	Return Value:
**		
**
**	Description:
**		This function transmits all the characters from a zero terminated string over UART4. The terminator character is not sent.
**      
**          
*/
void UART_PutString3(char szData[])
{
    char *pData = szData;
    while(*pData)
    {
        UART_PutChar3((*(pData++)));
    }
}
/****************************************
Same as putc3() but  w/o hardware control
*****************************************/
int putc3(char c)
{
   while ( U3STAbits.UTXBF);   // wait while Tx buffer is still full or  while(U3STAbits.TRMT == 0);
   U3TXREG = c;
   return c;
}
/************************* end of UART3 section ***************************/

/****************************** Uart5 section *****************************/

/***	UART_ConfigureUart
**
**	Parameters:
**		unsigned int baud - UART baud rate.
**                                     for example 115200 corresponds to 115200 baud
**
**	Return Value:
**		
**
**	Description:
**		This function configures the UART4 hardware interface of PIC32, according 
**      to the provided baud rate, no parity and 1 stop bit, with no interrupts.
**      In order to compute the baud rate value, it uses the peripheral bus frequency definition (PB_FRQ, located in config.h)
**      This is a low-level function called by initialization functions, so user should avoid calling it directly.   
**      
**          
*/

void UART5_ConfigureUart(unsigned int baud)
{
    U5MODEbits.ON     = 0;
    U5MODEbits.SIDL   = 0;
    U5MODEbits.IREN   = 0; 
    U5MODEbits.RTSMD  = 0;
    U5MODEbits.UEN0   = 0; 
    U5MODEbits.UEN1   = 0;
    U5MODEbits.WAKE   = 0;
    U5MODEbits.LPBACK = 0; 
    U5MODEbits.ABAUD  = 0;
    U5MODEbits.RXINV  = 0; 
    U5MODEbits.PDSEL1 = 0; 
    U5MODEbits.PDSEL0 = 0; 
    U5MODEbits.STSEL  = 0;  

    
    U5MODEbits.BRGH   = 0; 

    U5BRG = (int)(((float)PB_FRQ/(16*baud) - 1) + 0.5); // add 0.5 just in order to implement the round using the floor approach

    U5STAbits.UTXEN    = 1;
    U5STAbits.URXEN    = 1;
    U5MODEbits.ON      = 1; 
    
}


/***	UART_ConfigurePins
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function configures the digital pins involved in the UART module: 
**      The UART_TX digital pin is configured as digital output.
**      The UART_RX digital pin is configured as digital input.
**      The UART_TX and UART_RX are mapped over the UART interface.
**      The function uses pin related definitions from config.h file.
**      This is a low-level function called by UART_Init(), so user should avoid calling it directly.   
**          
*/
void UART5_ConfigurePins(){
   TRISEbits.TRISE8  = 0;   //TX digital output
   RPE8R = 0b0100;     // 0001 U3TX RPB5/RB5/AN5  shared with motor driver pin BIN2 
   // disable analog (set pins as digital))
   // ANSELBbits.ANSB5 = 0;
}


void uart5_init(int baud)
{
    UART5_ConfigurePins();
    UART5_ConfigureUart(baud);
}

/***	UART_PutChar
**
**	Parameters:
**          char ch -   the character to be transmitted over UART.
**
**	Return Value:
**		
**
**	Description:
**		This function transmits a character over UART4. 
**      
**          
*/
void UART_PutChar5(char ch)
{
    while(U5STAbits.UTXBF == 1);
    U5TXREG = ch;
}

/***	UART_PutString
**
**	Parameters:
**          char szData[] -   the zero terminated string containing characters to be transmitted over UART.
**
**	Return Value:
**		
**
**	Description:
**		This function transmits all the characters from a zero terminated string over UART4. The terminator character is not sent.
**      
**          
*/
void UART_PutString5(char szData[])
{
    char *pData = szData;
    while(*pData)
    {
        UART_PutChar5((*(pData++)));
    }
}
/****************************************
Same as putc3() but  w/o hardware control
*****************************************/
int putc5(char c)
{
   while ( U5STAbits.UTXBF);   // wait while Tx buffer is still full or  while(U3STAbits.TRMT == 0);
   U5TXREG = c;
   return c;
}

/************************* end of UART5 section ***************************/

#endif

#ifdef MICROSTICK_II
/* U1BRG (BRATE)
U2BRG = (PBCLK  / 16 / baudrate) -1 ; for BREGH=0
*/
// timing and baud rate calculations
//baud 115200
#define BRATE   21        // (40000000/16/115200)-1
 	
void Uart2_init( int baudrate)
{
   //U2BRG    = BRATE;  
   U2BRG = (40000000  / 16 / baudrate) -1 ; //for BREGH=0
   U2MODE    = 0x8000 ;
   U2STA    = 0x0400;      // enable transmission
  // TRTS    = 0;        // make RTS output
   //RTS     = 1;        // set RTS default status
} // initUart2


#endif



/*******************************************************************************
   Function: int  fprintf2(int mode, char *buffer){

   Precondition:
      uartx_init or lcd_init must be called prior to calling this routine.

   Overview:
        This function prints a string of characters to the selected console.
        Possible choices are C_UART1, C_UART2, C_LCD

   Input: 
        mode: select the console C_UART1, C_UART2 or  C_LCD
        buffer: Pointer to a character string to be outputted

   Output: returns the number of characters transmitted to the console

   *******************************************************************************/
int  fprintf2(int mode, char *buffer){
    int len =0;
    unsigned int i;
    char c;
    len = strlen(buffer);

    switch(mode){
        case C_UART1:  // not implemented for MX3
        /* Uart1 */
            for (i = len; i; --i){
#if defined EXPLORER_16_32
                UART1_Write(*(char*)buffer++);  
#endif 
            }
            return(len);
            break;
        /* Uart2 */
        case C_UART2:
                while(U2STAbits.TRMT == 0);  
                for (i = len; i; --i)
                {
                    while(U2STAbits.TRMT == 0);
                    U2TXREG = *(char*)buffer++;        
                }
                return(len);
            break;
        /* LCD */
#ifndef MICROSTICK_II
        case C_LCD:
            LCDL1Home();
            for (i = len; i; --i){
                c = *(char*)buffer++;
                if( c== '\n')LCDL2Home();
                else { 
#endif
#if defined EXPLORER_16_32
                LCDPut(c);
                                }
            }
            break;
#elif defined MX3
                LCD_WriteDataByte(c);
                                }
            }
            break;
#endif
                    


    }// switch case 
     return(len);
}


void set_stdio(int _stdio){
    stdio = _stdio;
}

// alias
void stdio_set(int _stdio){
    stdio = _stdio;
}

#ifdef RTOS
/*Only for RTOS system*/
static SemaphoreHandle_t mutex_stdio;
static int mutex_created =0;
/*
	Locks the stdio mutex.
	If the mutex does not exist yet, it will be created.
	Also, the function sets the stdio specified by the argument: 
		C_UART1 or C_UART2
*/
void stdio_lock(int _stdio){
    // creates the mutex only once
    if(mutex_created == 0){
        mutex_created = 1;
        mutex_stdio= xSemaphoreCreateMutex();
    }
    xSemaphoreTake(mutex_stdio, portMAX_DELAY);
    stdio = _stdio;
    return;

}
/*
	Unlocks the stdio mutex.
	If the mutex does not exist yet, the function immediately returns.
	Also, the function sets the stdio specified by the argument: 
		C_UART1 or C_UART2
*/
void stdio_unlock(int _stdio){
        // cannot unlock if mutex does not exist yet
        if(mutex_created == 0)return;  
        stdio = _stdio;
        xSemaphoreGive(mutex_stdio);
}

#endif
/*******************************************************************************

   Precondition:
      uartx_init or lcd_init must be called prior to calling this routine.

   Output: returns the number of characters transmitted to the console

*******************************************************************************/
void _mon_putc (char c)
{
    static int cur_line = 0;
    switch(stdio){
        case C_UART1:
        /* Uart1 */   
            while (U1STAbits.UTXBF);
            U1TXREG = c;
            break;
        /* Uart2 */
        case C_UART2:
            while(U2STAbits.TRMT == 0);
            U2TXREG = c;        
            break;
        case C_UART3:
            while(U3STAbits.TRMT == 0);
            U3TXREG = c;        
            break;            
        /* Uart4 */
#ifndef MICROSTICK_II
        case C_UART4:
            while(U4STAbits.UTXBF == 1);
            U4TXREG = c;     
            break; 
        case C_UART5:
            while(U5STAbits.UTXBF == 1);
            U5TXREG = c;     
            break; 
        /* LCD */
        case C_LCD:
            if (c <= 13){ // a control character
                if( c== '\r'){
                    LCDL1Home();
                    cur_line = 0;
                    return;
                }
                else if( c== '\n'){
                    cur_line = ! cur_line;
                    if(cur_line)LCDL2Home();
                    else LCDL1Home();
                    return;
                }
            }
            else { // not a control character
#endif
#if defined EXPLORER_16_32
                LCDPut(c);
            } 
            break;
#elif defined MX3
                LCD_WriteDataByte(c);
            } 
            break;
#endif      

    }// switch case 
}
 
 
 void printUart2FromISR(char *str){
    while(*str != '\0'){
        while(U2STAbits.TRMT == 0);
        U2TXREG = *str; 
        str++;
    }
}

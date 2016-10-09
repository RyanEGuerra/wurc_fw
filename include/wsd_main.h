/*
 * wsd_main.h
 * 	Main function for the WSD project.
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 *   CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Created: April 23, 2013
 * Edited:	July 27, 2013
 * Authors:	Ryan E. Guerra (me@ryaneguerra.com)
 * 			Narendra Anand (nanand@rice.edu)
 */

/*
 * ======== Include Descriptions ==========
 *
 * stdlib.h			: C Standard Library - contains some string-parsing functions
 *
 * inc/hw_memmap.h 	: Macros defining the memory map of the Stellaris device. This includes
 * 				 	  defines such as peripheral base address locations such as
 * 				 	  GPIO_PORTF_BASE
 * inc/hw_types.h 	: Defines common types and macros such as tBoolean and HWREG(x).
 * inc/hw_ints.h 	: Macros that define the interrupt assignment on Stellaris.
 *
 * inc/lm3s5y36.h 	: LM3S5Y36 Register Definitions
 *
 * wsd_defines.h	: Global defines used in the wsd_usb_client project
 *
 * driverlib/debug.h 		: Macros for assisting debug of the driver library.
 * driverlib/gpio.h 		: Defines and macros for GPIO API of DriverLib. This includes
 * 							  API functions such as GPIOPinTypePWM and GPIOPinWrite.
 * driverlib/interrupt.h	: Prototypes for the NVIC Interrupt Controller Driver.
 * driverlib/pin_map.h 		: Mapping of peripherals to pins for all parts.
 * driverlib/rom.h			: Macros to facilitate calling functions in the ROM.
 * driverlib/sysctl.h 		: Defines and macros for System Control API of DriverLib.
 * 							  This includes API functions such as SysCtlClockSet and
 * 							  SysCtlClockGet.
 * utils/flash_pb.h			: Prototypes for the flash driver
 *
 * driverlib/uart.h		: Defines and Macros for the UART.
 * utils/uartstdio.h	: Prototypes for the UART console functions.
 *
 * usblib/usblib.h				: Main header file for the USB Library.
 * usblib/usbcdc.h				: Definitions used by Communication Device Class devices.
 * usblib/device/usbdevice.h	: types and definitions used during USB enumeration.
 * usblib/device/usbdcdc.h		: USBLib support for generic CDC ACM (serial) device.
 *
 * utils/ustdlib.h					: Prototypes for simple standard library functions.
 * usb/wsd_usb_serial_structs.h	: Data structures defining this USB CDC device.
 * usb/wsd_usb.c					: Functions supporting the USB CDC interface
 * gpio_periph/wsd_ssi_master.h	: Contains all prototypes for 4-pin master serial interface.
 * gpio_periph/wsd_gpio.h			: Contains all helper prototypes for functional GPIO pins.
 *
 * */

#include "stdlib.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "inc/lm3s5r36.h"

#include "wsd_defines.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
//#include "utils/flash_pb.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "usb/wsd_usb_serial_structs.h"
#include "usb/wsd_usb.h"
#include "wsd_lms6002d_lib.h"
#include "wsd_calib_tables.h"
#include "wsd_io.h"
#include "wsd_gpio.h"

#include "string.h"
#include "volo_term.h"


#ifndef __WSD_MAIN_H__
#define __WSD_MAIN_H__

// ================================================================================
// Compilation Checks & Warnings
// ================================================================================

// check for compilation flags set in wsd_defines.h
#ifndef __ENABLE_UART__
#error REG - This code cannot be compiled without UART support.
#endif //__ENABLE_UART__

#ifndef __ENABLE_USB__
#error REG - This code cannot be compiled without UART support.
#endif //__ENABLE_USB__

// ================================================================================
// Global Variables
// ================================================================================

// Buffers for storing opcode and operand received from terminal sources. Also a
// number of valid chars in the buffers to save time on zeroing them out.
static char pcOpCodeBuff[OPCODE_MAX_SIZE];
static char pcOperandBuff[OPERAND_MAX_SIZE];
volatile static int iNumCharsInCommandBuffer = 0;

// Flag to signal to the main loop that one of the terminal input sources has
// data ready for processing.
volatile static int iUART0BufferHasChars = 0;
volatile static int iUSB0BufferHasChars = 0;

// Flag to signal to the main loop that the terminal buffers should be flushed
// and the initialization message should be displayed.
volatile static char g_cResetTerminalPendingFlag = 0;

// Current state of external PA activation--used for debugging, and not useful
// anymore.
//volatile static char g_cPASelectToggleState = 0;

// Keeps track of the Tx/Rx switch state for toggling between them.
extern char g_cTxRxSwitchState;

// Counts the number of main loop iterations. Used for LED blink timing. :)
volatile static int g_iMainPollLoopCounter = 0;

// ================================================================================
// Function Prototypes
// ================================================================================
void UART0IntHandler(void);
void SetupUART0(void);

#ifdef __ENABLE_USB__
	void SetupUSB0(void);
#endif //__ENABLE_USB__

void SetupSSI0(void);
void GPIO_PortB_IntHandler(void);
void SetupFPGA_GPIO(void);
void SetupLEDs(void);
void SetupRFCtrl(void);
void Stel_ResetTerminalDevice(void);
void ClearCommandBuffers(void);
//unsigned long myHex2Long(char *cBuff, int iLength);
//unsigned long myDec2Long(char *cBuff);
//void myLong2Hex(unsigned long ulVal, char *cBuff, int iLength);
//unsigned long Print(const unsigned char *pucStr, unsigned long ulCharsToWrite);
//unsigned long PrintUSB(const unsigned char *pucStr, unsigned long ulCharsToWrite);
//unsigned long PrintUART(const unsigned char *pucStr, unsigned long ulCharsToWrite);
//void PrintStrUSB(char *pucStr);
//void PrintStrUART(char *pucStr);
//void PrintStr(const unsigned char *pucStr);
unsigned long PrintStrUARTDebug(const unsigned char *pucStr);
unsigned long PrintStrUSBDebug(const unsigned char *pucStr);
void Stel_PrintHelpDialog(void);
void Stel_PrintStatusJSON(TerminalBuff_t* pBuff);
//void EchoUART(void);
void Stel_ProcessUSB0Input();
void Stel_ProcessUART0Input();
extern void volo_executeCommand(TerminalBuff_t* pBuff);
//void Stel_ExecuteCommand(char ucOpCode, char *ucOperand);
void Stel_SetUSB0BufferHasCharsFlag();
void Stel_SetResetTerminalPendingFlag();

//int Stel_InitFlashCalibrationTable(void);
//CalTable_t * Stel_GetCalibrationTable(void);
//int Stel_SetCalibrationTable(CalTable_t * psNewTable);
//int Stel_AtomicCommitCalibrationTable();
//void Stel_PrintCalibrationTable(int band_no);

//void Stel_PrintReg(uint32_t regval, char *pcBuff);
//uint32_t Stel_hexSubstrToLong(char * ucStr, uint32_t startInd, uint32_t stopInd);

//void Stel_TurnOnErrorLED(void);
//void Stel_TurnOffErrorLED(void);
//void Stel_TurnOnHeartbeatLED(void);
//void Stel_TurnOffHeartbeatLED(void);

int main(void);

#endif //__WSD_MAIN_H__

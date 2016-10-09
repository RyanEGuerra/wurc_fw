/*
 * wsd_io.h
 *
 *  Created on: Feb 22, 2014
 *      Author: rng
 */

#include "stdlib.h"
#include "driverlib/rom.h"

#include "usb/wsd_usb_serial_structs.h"
#include "wsd_defines.h"
#include "volo_term.h"
#include "wsd_lms6002d_lib.h"

#ifndef WSD_IO_H_
#define WSD_IO_H_

// Because dual-terminal functionality was added later, the main code
// is not aware that there are two interfaces to print stuff to. This
// pointer lets us register which terminal is the current active terminal.
// When a command is executed, the originating terminal is registered here;
// since all commands are "REST-ful," this means that the correct terminal
// is always registered when output needs to be printed. Natch!
extern TerminalBuff_t *g_activeTermBuff;

// =============================================================================
// Macro for inserting a new line on either interface. We have this just in case
// we want to change the newline character.
// ...and it's a good thing we did! Because now integrating two terminals just
// because a lot easier, so this needs to be updated. REG - Aug 2013
#define __newline PrintStr("\n\r");
// =============================================================================

unsigned long Print(const unsigned char *pucStr, unsigned long ulCharsToWrite);
unsigned long PrintUSB(const unsigned char *pucStr, unsigned long ulCharsToWrite);
unsigned long PrintUART(const unsigned char *pucStr, unsigned long ulCharsToWrite);

void PrintStr(const unsigned char *pucStr);
void PrintStrUSB(char *pucStr);
void PrintStrUART(char *pucStr);

void myLong2Hex(unsigned long ulVal, char *cBuff, int iLength);

void Stel_PrintReg(uint32_t regval, char *pcBuff);
void Stel_PrintReg_DEBUG(uint32_t regval);
uint32_t Stel_hexSubstrToLong(char * ucStr, uint32_t startInd, uint32_t stopInd);
unsigned long myHex2Long(char *cBuff, int iLength);

void Stel_ThrowError(const unsigned char *pucStr);

unsigned long myDec2Long(char *cBuff);

//**************************************************************
/* Helper macros */
#define HEX__(n) 0x##n##LU
#define B8__(x) ((x&0x0000000FLU)?1:0) \
+((x&0x000000F0LU)?2:0) \
+((x&0x00000F00LU)?4:0) \
+((x&0x0000F000LU)?8:0) \
+((x&0x000F0000LU)?16:0) \
+((x&0x00F00000LU)?32:0) \
+((x&0x0F000000LU)?64:0) \
+((x&0xF0000000LU)?128:0)

/* User macros */
#define B8(d) ((unsigned char)B8__(HEX__(d)))
#define B16(dmsb,dlsb) (((unsigned short)B8(dmsb)<<8) \
+ B8(dlsb))
#define B32(dmsb,db2,db3,dlsb) (((unsigned long)B8(dmsb)<<24) \
+ ((unsigned long)B8(db2)<<16) \
+ ((unsigned long)B8(db3)<<8) \
+ B8(dlsb))


//#include <stdio.h>

//int main(void)
//{
//    // 261, evaluated at compile-time
//    unsigned const number = B16(00000001,00000101);
//
//    printf("%d \n", number);
//    return 0;
//}

#endif /* WSD_IO_H_ */

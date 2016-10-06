/*
 * wsd_io.c
 *
 *  Created on: Feb 22, 2014
 *      Author: rng
 */


#include "include/wsd_io.h"

// Because dual-terminal functionality was added later, the main code
// is not aware that there are two interfaces to print stuff to. This
// pointer lets us register which terminal is the current active terminal.
// When a command is executed, the originating terminal is registered here;
// since all commands are "REST-ful," this means that the correct terminal
// is always registered when output needs to be printed. Natch!
TerminalBuff_t *g_activeTermBuff;


//*****************************************************************************
//
// Prints a given number of characters from the passed char* to a given
// interface. This helper function is required to make code interface-
// neutral. Relies on preprocessor variables to determine which interface
// is used.
//
// \param ucStr The string to print.
// \param ulCharsToWrite The number of characters to actually print.
//
// \return The number of chars actually written to the interface.
//
//*****************************************************************************
unsigned long
Print(const unsigned char *pucStr, unsigned long ulCharsToWrite)
{
	unsigned long ulNumWritten = 0;

#ifdef __ENABLE_USB__
	ulNumWritten = PrintUSB(pucStr, ulCharsToWrite);
#endif //__ENABLE_USB

#ifdef __ENABLE_UART__
	ulNumWritten = PrintUART(pucStr, ulCharsToWrite);
#endif //ENABLE_UART

	return ulNumWritten;
}

//*****************************************************************************
//
// Prints a given number of characters from the passed char* to the USB0 iface.
// Print ulCharsToWrite number of chars or until the end of the string, whichever
// is first.
// Note that this function currently blocks if the USB0 TX buffer is full. This
// is not safe, but fixing it is annoying.
//
// \param ucStr The string to print.
// \param ulCharsToWrite The number of characters to actually print.
//
// \return The number of chars actually written to the USB0 buffer.
//
//*****************************************************************************
unsigned long
PrintUSB(const unsigned char *pucStr, unsigned long ulCharsToWrite)
{
	unsigned long ulCharsWritten = 0;
	unsigned long ulSpace;
	unsigned long usbTimeoutCount = 0;
	char usbIsStuck = FALSE;

	ulSpace = USBBufferSpaceAvailable(&g_sTxBuffer);

	// Read data from the string buffer until there is none left or there is no
	// more space in the USB TX buffer, and add them to the USB TX buffer.
	// Also exit if we've reached the end of the string!
	while((ulCharsToWrite - ulCharsWritten > 0) && (pucStr[ulCharsWritten] != '\0'))
	{
		// block until space opens up.
		//todo - come back and optimize this

		while (ulSpace == 0)
		{
			ulSpace = USBBufferSpaceAvailable(&g_sTxBuffer);
			// TODO - make this safe--we tried, but it's a bit buggy and we
			// fixed the problem in other ways
			if (++usbTimeoutCount > USB_TIMEOUT_MAX_COUNT)
			{
				usbIsStuck = TRUE;
				break;
			}
		}

		//Reset the timeout, since we made it outs
		usbTimeoutCount = 0;

		if (usbIsStuck)
		{
			// TODO - come back and fix properly
			ulCharsWritten = 10;
			break;
		}

		// get the next char in the buffer and write it
		USBBufferWrite(&g_sTxBuffer, pucStr + ulCharsWritten, 1);
		ulCharsWritten++;

		// check the remaining space available to the USB buffer
		ulSpace = USBBufferSpaceAvailable(&g_sTxBuffer);
	}

	return ulCharsWritten;
}

//*****************************************************************************
//
// Prints a given number of characters from the passed char* to the UART0 iface.
//
// \param ucStr The string to print.
// \param ulCharsToWrite The number of characters to actually print.
//
// \return The number of chars actually written to the USB0 buffer.
//
//*****************************************************************************
unsigned long
PrintUART(const unsigned char *pucStr, unsigned long ulCharsToWrite)
{
	unsigned long ulCharsWritten = 0;

		// While there are more Chars to write or until the string ends
		while((ulCharsToWrite - ulCharsWritten > 0) && (pucStr[ulCharsWritten] != '\0'))
		{
			// If there is space, write more chars to the buffer!
			//TODO - this is not safe; check to see if there is another way
			if (ROM_UARTSpaceAvail(UART0_BASE))
			{
				ROM_UARTCharPutNonBlocking(UART0_BASE, pucStr[ulCharsWritten]);
				ulCharsWritten++;
			}
		}

	return ulCharsWritten;
}

//*****************************************************************************
//
// Prints the entirety of the passed char* to the currently active iface.
// Note that USB0print currently blocks if the USB0 TX buffer is full. This
// is not safe, but fixing it is annoying.
//
// \param ucStr The string to print.
//
//*****************************************************************************
void
PrintStr(const unsigned char *pucStr)
{
	__terminalPrint(g_activeTermBuff, (char *)pucStr);

	return;
}

//*****************************************************************************
// Stripped down USB print function that accepts a null-terminated string.
// Rather than some other functions that let us specify the length of
// the string to print.
//
// \param pucStr The null-terminated string to print to the USB interface.
//*****************************************************************************
void
PrintStrUSB(char *pucStr)
{
	unsigned long ulStrLen;

	ulStrLen = strlen(pucStr);
	PrintUSB((unsigned char*)pucStr, ulStrLen);
}

//*****************************************************************************
// Stripped down UART print function that accepts a null-terminated string.
// Rather than some other functions that let us specify the length of
// the string to print.
//
// \param pucStr The null-terminated string to print to the UART interface.
//*****************************************************************************
void
PrintStrUART(char *pucStr)
{
	unsigned long ulStrLen;

	ulStrLen = strlen(pucStr);
	PrintUART((unsigned char*)pucStr, ulStrLen);
}

//*****************************************************************************
//
// I'm kinda annoyed, but the built-in atol doesn't seem to handle the number
// zero properly. It also seems to SUCK. So this catches that case. WTF.
// I'm a terrible coder and this is probably the worst way to do this.
// TI, I am disappoint.
//
// \param ulVal The unsigned long value to convert to a hex string.
// \param cBuff A character buffer to placed the formatted hex string into.
// \param iLength The minimum length of the string to print out. Note this is
//                ONLY a minimum! Make sure the buffer is large enough.
//
//*****************************************************************************
void
myLong2Hex(unsigned long ulVal, char *cBuff, int iLength)
{
	unsigned long ulTest;
	int jj;
	int offset = 0;

	for(jj = 0; jj < 8; jj++)
	{
		// mask out the top 4 bits, then bit shift to get values [0-F]
		ulTest = (ulVal & 0xF0000000) >> 28;

		// whenever we've already printed the non-zero MSD or we're hitting
		// the first non-zero MSD, we print a character. We also force a
		// character to be printed if this is the LOWEST 4 bits since we
		// have a valid char output when ulVal is zero
		if (offset > 1 || ulTest > 0 || jj >= (8-iLength))
		{
			if (ulTest < 10)
			{
				// 48 is the ASCII offset for [0-9]
				cBuff[offset] = (char) (ulTest + 48);
			}
			else
			{
				// ASCII A = 65; A = 10; so ASCII offset for [A-F] is 55
				cBuff[offset] = (char) (ulTest + 55);
			}
			// move on to the next character
			offset++;
		}

		// check the next 4 bits
		ulVal = ulVal << 4;
	}
	// don't forget the null terminator!
	cBuff[offset] = '\0';

	return;
}

//*****************************************************************************
//
// Print the passed 32-bit register value to the current active terminal.
// Format is: 0x1234.5678
//
// \param regval the value to print
// \param a pointer to a character buffer that is at least 5-chars wide
//
//*****************************************************************************
void
Stel_PrintReg(const uint32_t regval, char *pcBuff)
{
	PrintStr("0x");
	myLong2Hex( (regval & 0xFFFF0000) >> 16, pcBuff, 4);
	PrintStr((unsigned char *)pcBuff);
	PrintStr(".");
	myLong2Hex( (regval & 0x0000FFFF), pcBuff, 4);
	PrintStr((unsigned char *)pcBuff);
}

//*****************************************************************************
//
// Print the passed 32-bit register value to the current active terminal.
// Format is: 0x1234.5678
//
// \param regval the value to print
// \param a pointer to a character buffer that is at least 5-chars wide
//
//*****************************************************************************
void
Stel_PrintReg_DEBUG(uint32_t regval)
{
	char pcBuff[6];
	unsigned long ulBuff;

	Lime_Read(regval, &ulBuff);

	PrintStr("0x");
	myLong2Hex( (ulBuff & 0xFFFF0000) >> 16, pcBuff, 4);
	PrintStr((unsigned char *)pcBuff);
	PrintStr(".");
	myLong2Hex( (ulBuff & 0x0000FFFF), pcBuff, 4);
	PrintStr((unsigned char *)pcBuff);
}

//*****************************************************************************
// Function to convert a specified substring to a long and return that value.
//
// \param ucStr the original string to parse
// \param startInd the starting index of the substring
// \param stopInd the ending index of the substring; the converted substring
//                will INCLUDE this index
//
// \return integer representation of the parsed substring
//*****************************************************************************
uint32_t
Stel_hexSubstrToLong(char * pcStr, uint32_t startInd, uint32_t stopInd)
{
	char pcBuff[20];
	int ii;

	for (ii = 0; ii <= stopInd - startInd; ii++)
	{
		pcBuff[ii] = pcStr[startInd + ii];
	}
	// Don't need to null terminate b/c hex string have explicit length.
//	pucBuff[stopInd - startInd + 1] = '\0';
	return myHex2Long(pcBuff, stopInd - startInd + 1);
}

//*****************************************************************************
//
// The ltoa function is also somewhat under-whelming. This is a lazy hack.
// Converts UPPER case HEX strings to a unsigned long.
//TODO improve or remove this ltoa hack.
//
// \param cBuff The string to convert to an unsigned long value. MUST be in HEX
//              format. This function assumes the string is in upper-case HEX.
// \param iLength The exact length of the string to convert. If the string is
//                longer, we will only try to convert the first iLength chars.
//
// \return An unsigned long containing the converted data value.
//
//*****************************************************************************
unsigned long
myHex2Long(char *cBuff, int iLength)
{
	unsigned long ulReturn = 0;
	unsigned long ulTemp;
	int index;

	// Rationality check
	if (iLength <= 0)
	{
		Stel_ThrowError("16");
		return 0;
	}

	// until we hit the end of this string...
	for (index = iLength - 1; index >= 0; index--)
	{
		ulTemp = (unsigned long) cBuff[index];
		// ASCII 0 is 48, ASCII 9 is 57
		// ASCII A is 65, ASCII F is 70
		if (ulTemp < 48 || ulTemp > 70)
		{
			Stel_ThrowError("Non-numeric Char");
			return 0;
		}
		// value is A-F
		if (ulTemp > 57)
		{
			ulTemp -= 55;
		}
		// value is 0-9
		else
		{
			ulTemp -= 48;
		}
		// a little complex, just shift the temp variable to the right bit
		// location and add the value to the running sum.
		ulReturn = ulReturn + (ulTemp << ((iLength-1-index)*4));
	}

	return ulReturn;
}

//*****************************************************************************
// Main error handling function; wraps the error msg in "!" chars for external
// processing and also turns the error LED to active.
//
// \param The error description string to print to UART.
//*****************************************************************************
void
Stel_ThrowError(const unsigned char *pucStr)
{
	PrintStr("! ");
	PrintStr(pucStr);
	PrintStr(" !");
	__newline

	Stel_TurnOnErrorLED();
}

//*****************************************************************************
//
//TODO - bounds checking, also make sure it's actually a number
//
//*****************************************************************************
unsigned long
myDec2Long(char *cBuff)
{
	const int iLen = strlen(cBuff);
	int jj;
	unsigned long ulBuff;
	unsigned long ulIndex, ulPow, ulCoEff;
//	char cDebugBuff[10];

	// Rationality check
	if (iLen <= 0)
	{
		Stel_ThrowError("17");
		return 0;
	}

	// Get the LSD
	ulBuff = myHex2Long(cBuff+iLen-1, 1);
	for (ulIndex = 0; ulIndex < iLen-1; ulIndex++)
	{
		// 1, 2, 3, etc... not 0; that's handled above
		ulPow = (unsigned long)iLen - ulIndex -  (unsigned long)1;
		ulCoEff = 10;
		for (jj = 1; jj < ulPow; jj++)
		{
			ulCoEff *= 10;
		}
		// Starting from the MSD, keep adding in the contribution
		ulBuff += ulCoEff * myHex2Long(cBuff+ulIndex, 1);

//		PrintStr("> ");
//		myLong2Hex(ulBuff, cDebugBuff, 1);
//		PrintStr((unsigned char *)cDebugBuff);
//		__newline
	}

	return ulBuff;
}

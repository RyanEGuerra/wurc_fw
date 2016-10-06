/*
 * bb_interface_lib.c
 *
 * Contains functions to interface with the radio card host baseband. For now, it's WARP
 * but in the future, it may be another board. In the latter case, these functions have
 * to change, but the prototypes can remain the same.
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 *   CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  Created on: July 28, 2012
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 */

#include "inc/hw_types.h"

#include "include/bb_interface_lib.h"
#include "include/wsd_defines.h"
#include "include/wsd_main.h"


//*****************************************************************************
//
// Test the connection between the Stellaris Microcontroller and the host BB UART.
//
// \return Zero if the connection responds correctly. One if connection problem.
//
//*****************************************************************************
int
BB_TestBBConnection(void)
{
	//TODO test connection between Stellaris and host BB
	return 1;
}

//*****************************************************************************
//
// Set baseband I/Q Calibration values for the base address.
//
// \param ulBaseAddr the target configuration register to write to
// \param ulIQCalValue a 32-bit value interpreted as a Fix_0_32 to write to register
//
// \return zero if successful, or non-zero if error occurred
//
//*****************************************************************************
int
BB_SetIQCalibration(unsigned long ulBaseAddr, unsigned long ulIQCalValue)
{
	// Don't write zeros to the gain multipliers!
	if( (ulBaseAddr == BB_IQCAL_TX_GAINMULT_R ||
		 ulBaseAddr == BB_IQCAL_RX_GAINMULT_R ||
		 ulBaseAddr == BB_IQCAL_TX_COSMULT_R  ||
		 ulBaseAddr == BB_IQCAL_RX_COSMULT_R)
		&& (ulIQCalValue == 0) )
	{
		// If the GainMult or CosMult cal value is zero, then error and don't write it.
		// There is no valid case where we would actually want to write a zero.
		PrintStr("Cal Gain is Zero");
		__newline
		return 0;
	}

	if (BB_UARTSendLongWord(ulBaseAddr, ulIQCalValue))
	{
		// Error occured with BB connection
		return 1;
	}
	__NOP_DELAY_LOOP(10000);
	return 0;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
int
BB_SetHostSerialNumber(unsigned long ulSerialNumber)
{
	if (BB_UARTSendLongWord(BB_SERIAL_CODE_R, ulSerialNumber))
	{
		// Error occured with BB connection
		return 1;
	}
	return 0;
}

//*****************************************************************************
//
// Controls what samples are input to the ADC of the LMS6002D. This controls
// then input to the daughtercard from the WARP firmware.
//
// \param state the state to set the DAC input to; {ALLZERO, NORMAL}
//
// \return a zero upon success, non-zero otherwise.
//
//*****************************************************************************
int
BB_SetDACInputState(DAC_Input_t state)
{
	int errors = 0;

	if (state == ALLZERO)
	{
		// Set all input DAC samples to zero.
		errors += BB_UARTSendCommand(WARP_ADCINPUT_ALLZEROS);
	}
	else if (state == NORMAL)
	{
		// Restore DAC sample state by restoring gmult value
		errors += BB_UARTSendCommand(WARP_ADCINPUT_NORMAL);
	}
	else
	{
		errors++;
	}

	// Arbitrary delay to let the baseband react to the command before
	// continuing.
	__NOP_DELAY_LOOP(1000000);

	return errors;
}

//*****************************************************************************
//
// Sends a long word (32-bits, or 8 bytes) via UART baseband connection.
//
// \param ulAddr the target configuration register to write to
// \param ulLongData a 32-bit value to write to register
//
// \return zero if successful, or non-zero if error occurred
//
//*****************************************************************************
int
BB_UARTSendLongWord(unsigned long ulAddr, unsigned long ulLongData)
{
	char pcBuff[10];

	// The tilde serve as "escape characters"
	// ASCII 126
	pcBuff[0] = '~';
	pcBuff[1] = '\0';
	PrintStrUARTDebug((unsigned char *)pcBuff);

	myLong2Hex(ulAddr, pcBuff, 2);
	PrintStrUARTDebug((unsigned char *)pcBuff);
	myLong2Hex(ulLongData, pcBuff, 8);
	PrintStrUARTDebug((unsigned char *)pcBuff);

	// The tilde serve as "escape characters"
	// ASCII 126
	pcBuff[0] = '~';
	pcBuff[1] = '\0';
	PrintStrUARTDebug((unsigned char *)pcBuff);

	return 0;
}

//*****************************************************************************
//
//	Send a single command word to the WARP firmware via the UART interface.
//
// \param ulCmdWord the command to send to the BB processor (WARP)
//
// \return a zero upon success.
//
//*****************************************************************************
int
BB_UARTSendCommand(unsigned long ulCmdWord)
{
	char pcBuff[10];

	// The tilde serve as "escape characters"
	// ASCII 126
	pcBuff[0] = '~';
	pcBuff[1] = '\0';
	PrintStrUARTDebug((unsigned char *)pcBuff);

	myLong2Hex(ulCmdWord, pcBuff, 2);
	PrintStrUARTDebug((unsigned char *)pcBuff);

	// The tilde serve as "escape characters"
	// ASCII 126
	pcBuff[0] = '~';
	pcBuff[1] = '\0';
	PrintStrUARTDebug((unsigned char *)pcBuff);

	return 0;
}


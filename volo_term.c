/*
 * volo_term.c
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VOLO WIRELESS, LLC SHALL NOT,
 *   UNDER ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  Created on: August 16, 2013
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 */

#include "include/volo_term.h"

/*
 * Initialize a terminal buffer.
 *
 * \param pBuff pointer to a defined term_buff_t struct
 * \param printChar function pointer to a function that prints chars to the host
 */
void
volo_initTermBuff(TerminalBuff_t* pBuff, void (*printChar)(char *))
{
	int ii;

	pBuff->numCharsInBuff = 0;
	// Clear opcode buffer
	for (ii = 0; ii < VOLO_TERM_OPSIZE + 1; ii++)
	{
		pBuff->cOpCodeBuff[ii] = '\0';
	}
	// Clear command buffer
	for (ii = 0; ii < VOLO_TERM_BUFFERSIZE + 1; ii++)
	{
		pBuff->cCmdBuff[ii] = '\0';
	}
	// Pointer to the terminal printChar function (to allow
	// buffers to "attach" to different UART/IO sources/sinks)
	// Ex: (*(pBuff->vTermPrintCharCallback))(charPtrToPrint)
	pBuff->vTermPrintCharCallback = printChar;

	// Print the prompt to signal it's ready to take input.
	volo_printPrompt(pBuff);
}

/*
 * Clear the command buffer. We could print something if we wanted toOh!FrabjousDay!
 *
 */
void
volo_resetCommandBuffer(TerminalBuff_t* pBuff)
{
	pBuff->numCharsInBuff = 0;
	// Null both opcode chars - this makes comparisons later on easier.
	pBuff->cOpCodeBuff[0] = '\0';
	pBuff->cOpCodeBuff[1] = '\0';
	// To save time, we only make the first char in the buffer null.
	pBuff->cCmdBuff[0] = '\0';

	return;
}

/*
 * Processes a new input character for the passed buffer.
 *
 * \param pBuff a pointer to a term_buff_t struct associated with a terminal interface
 * \param newChar the new char to process
 */
void
volo_processChar(TerminalBuff_t* pBuff, char newChar)
{
	char pCharBuff[2];

	pCharBuff[0] = newChar;
	pCharBuff[1] = '\0';

	// Test for special characters input into the buffer.
	switch (newChar)
	{
		case ASCII_CR:
		{
			volo_executeCommand(pBuff);
			volo_resetCommandBuffer(pBuff);
			volo_printPrompt(pBuff);
			break;
		}
		case ASCII_ESC:
		{
			volo_resetCommandBuffer(pBuff);
			volo_printPrompt(pBuff);
			break;
		}
		case OP_MACHINE_JSON:
		{
			// Print nothing but the JSON state string to the terminal
			Stel_PrintStatusJSON(pBuff);
			break;
		}
		default:
		{
			// Based on the number of chars in the buffer, we do different
			// things in order to print it properly.
			if (pBuff->numCharsInBuff < VOLO_TERM_OPSIZE)
			{
				// Test if this is a quick opcode, or regular opcode. The first
				// OpCode character determines this. If numChars is zero,
				// the buffer is completely empty
				switch (newChar)
				{
					case ASCII_DEL:
					{
						if (pBuff->numCharsInBuff != 0)
						{
							// ignore BS when buffers are empty
							__terminalPrint(pBuff, "\b \b");
							pBuff->numCharsInBuff--;
							pBuff->cOpCodeBuff[pBuff->numCharsInBuff] = '\0';
						}
						break;
					}
					case ASCII_0:
					case ASCII_1:
					case ASCII_2:
					case ASCII_3:
					case ASCII_4:
					case ASCII_5:
					case ASCII_6:
					case ASCII_7:
					case ASCII_8:
					case ASCII_9:
						// Check to see if this is a quick command (e.g. it's
						// the first char in the OpCode.)
						if (pBuff->numCharsInBuff == 0)
						{
							// Print the number to the terminal, for verification
							__terminalPrint(pBuff, pCharBuff);
							pBuff->cOpCodeBuff[0] = newChar;
							pBuff->numCharsInBuff++;
							volo_executeCommand(pBuff);
							volo_resetCommandBuffer(pBuff);
							volo_printPrompt(pBuff);
							break;
						}
						//else, fallthrough
					default:
					{
						// Add to the opcode buffer
						pBuff->cOpCodeBuff[pBuff->numCharsInBuff] = newChar;
						pBuff->numCharsInBuff++;
						// Print the buffered char to terminal and a space if
						// opcode is complete in order to signal correct parsing
						__terminalPrint(pBuff, pCharBuff);
						if (pBuff->numCharsInBuff == VOLO_TERM_OPSIZE)
							__terminalPrint(pBuff, " ");
						break;
					}
				} //switch (newChar)
			}
			else if (pBuff->numCharsInBuff < VOLO_TERM_BUFFERSIZE + VOLO_TERM_OPSIZE)
			{
				// This means that the opcode has been entered, and the command
				// is now being entered.
				if (newChar == ASCII_DEL)
				{
					if (pBuff->numCharsInBuff == VOLO_TERM_OPSIZE)
					{
						// Two deletes b/c of the extra space printed after the opcode
						__terminalPrint(pBuff, "\b\b \b");
						// Need to clear the char in the OpCode Buffer
						pBuff->numCharsInBuff--;
						pBuff->cOpCodeBuff[pBuff->numCharsInBuff] = '\0';
					}
					else
					{
						__terminalPrint(pBuff, "\b \b");
						// Need to clear the char in the Command Buffer
						pBuff->cCmdBuff[pBuff->numCharsInBuff - VOLO_TERM_OPSIZE - 1]
						                = '\0';
						pBuff->numCharsInBuff--;
					}
				}
				else
				{
					// Add to the opcode buffer; null-terminate the current string
					// so that it prints correctly.
					pBuff->cCmdBuff[pBuff->numCharsInBuff - VOLO_TERM_OPSIZE]
									= newChar;
					pBuff->cCmdBuff[pBuff->numCharsInBuff - VOLO_TERM_OPSIZE + 1]
														= '\0';
					// Print the buffered char to terminal
					__terminalPrint(pBuff, pCharBuff);
					pBuff->numCharsInBuff++;
				}
			}
			else
			{
				// Do not accept any more characters. But allow the user to delete
				// them!
				if (newChar == ASCII_DEL)
				{
					__terminalPrint(pBuff, "\b \b");
					// Need to clear the char in the Command Buffer
					pBuff->cCmdBuff[pBuff->numCharsInBuff - VOLO_TERM_OPSIZE - 1]
									= '\0';
					pBuff->numCharsInBuff--;
				}
			}
			break;
		}
	} //switch (newChar)

//	char numBuff[2];
//	numBuff[0] = (char)(pBuff->numCharsInBuff + 48);
//	numBuff[1] = '\0';

	// Debug stuff. TODO - remove this
//	__terminalPrint(pBuff, "\r\n OP: ");
//	__terminalPrint(pBuff, pBuff->cOpCodeBuff);
//	__terminalPrint(pBuff, "\r\nCMD: ");
//	__terminalPrint(pBuff, pBuff->cCmdBuff);
//	__terminalPrint(pBuff, "\r\nLEN:" );
//	__terminalPrint(pBuff, numBuff);

	return;
}

/*
 * Print the command prompt and any additional command-end processing.
 */
void
volo_printPrompt(TerminalBuff_t* pBuff)
{
	__terminalPrint(pBuff, "\r\nwss$ ");
	return;
}

/*
 * Utility function to test the length of a command string against a
 * desired length
 */
int
util_test_cmd_len(TerminalBuff_t *pBuff, int numExpected)
{
	if(strlen(pBuff->cCmdBuff) != numExpected)
	{
		__terminalPrint(pBuff, "\r\n! Bad Argument Length !");
		__terminalPrint(pBuff, "\r\n  OP: ");
		__terminalPrint(pBuff, pBuff->cOpCodeBuff);
		__terminalPrint(pBuff, "\r\n CMD: ");
		__terminalPrint(pBuff, pBuff->cCmdBuff);
		return 1;
	}
	return 0;
}

/*
 * Converts a decimal Fix9_0 number (say, one read from shared registers)
 * to a number string and loads it in the passed string buffer.
 */
void
util_fix9_0_to_str(uint32_t val, char *pcBuff)
{
	if (val > 257)
	{
		// The value is negative. Convert from 2s-complement
		pcBuff[0] = '-';
		// Twos complement translation
		val = (~val) + 1;
		// Mask out higher bits > 9
		val = val & 0x000001FF;
		snprintf(pcBuff+1, 8, "%d", val);
	}
	else
	{
		// The value is positive and can be printed as-is.
		// Mask out higher bits > 10, just in case
		val = val & 0x000001FF;
		//is positive
		snprintf(pcBuff+1, 8, "%d", val);
	}
	return;
}

/*
 * Converts a decimal Fix10_0 number (say, one read from shared registers)
 * to a number string and loads it in the passed string buffer.
 */
void
util_fix10_0_to_str(uint32_t val, char *pcBuff)
{
	if (val > 513)
	{
		// The value is negative. Convert from 2s-complement
		pcBuff[0] = '-';
		// Twos complement translation
		val = (~val) + 1;
		// Mask out higher bits > 10
		val = val & 0x000003FF;
		// Note the +1 because the first character is the neg sign
		snprintf(pcBuff+1, 8, "%d", val);
	}
	else
	{
		// The value is positive and can be printed as-is.
		// Mask out higher bits > 10, just in case
		val = val & 0x000003FF;
		// No negative sign needed
		snprintf(pcBuff, 8, "%d", val);
	}
	return;
}

/*
 * Converts a string into the binary representation of a Fix9_0
 */
uint32_t
util_str_to_fix9_0(char *pcBuff)
{
	uint32_t val;

	if (pcBuff[0] == '-')
	{
		// The value is negative and we must convert to 2s complement
		// Skip the first char in the conversion beause it's the negative sign.
		val = atoi(pcBuff + 1);
		// Twos complement change
		val = (~val) + 1;
		// Unnecessary, but make this explicit: only the first 10 bits matter.
		val = val & 0x000001FF;
	}
	else
	{
		// The value is positive and we can just convert as-is w/ error checking
		val = atoi(pcBuff);
		// Unnecessary, but make this explicit: only the first 10 bits matter.
		val = val & 0x000001FF;
	}
	return val;
}

/*
 * Converts a string into the binary representation of a Fix10_0
 */
uint32_t
util_str_to_fix10_0(char *pcBuff)
{
	uint32_t val;

	if (pcBuff[0] == '-')
	{
		// The value is negative and we must convert to 2s complement
		// Skip the first char in the conversion beause it's the negative sign.
		val = atoi(pcBuff + 1);
		// Twos complement change
		val = (~val) + 1;
		// Unnecessary, but make this explicit: only the first 10 bits matter.
		val = val & 0x000003FF;
	}
	else
	{
		// The value is positive and we can just convert as-is w/ error checking
		val = atoi(pcBuff);
		// Unnecessary, but make this explicit: only the first 10 bits matter.
		val = val & 0x000003FF;
	}
	return val;
}

/*
 * Takes a hex string and returns a number representing that hex string.
 * Processes the first numChars hex digits in the string, or assumes that
 * it is zero-padded if the strlen is smaller than the numChars.
 *
 * If numChars is > 8, only converts the first 8 chars.
 *
 * Returns 0 if any non-numeric characters are encountered.
 */
uint32_t
util_hexStr_to_int(char *pcBuff, size_t numChars)
{
	int ii;
	uint32_t retVal = 0;

	// Maximum number of chars is 8 (32 bits)
	if (numChars > 8)
	{
		numChars = 8;
	}

	// Test length of string.
	if (strlen(pcBuff) < numChars)
	{
		numChars = strlen(pcBuff);
	}

	// Test each of the characters for their numeric value and
	// shift them into the return value, 8 bits at a time.
	// Note: you start with the LSB, which is index [numChars-1].
	for (ii = numChars-1; ii >= 0 ; ii--)
	{
		// Test if digit is [A-F]
		if ( (pcBuff[ii] >= 65) && (pcBuff[ii] <= 70) )
		{
			retVal += pcBuff[ii] - 55;
		}
		// Test if digit is [a-f]
		else if ( (pcBuff[ii] >= 97) && (pcBuff[ii] <= 102) )
		{
			retVal += pcBuff[ii] - 87;
		}
		// Test if digit is [0-9]
		else if ( (pcBuff[ii] >= 48) && (pcBuff[ii] <= 57) )
		{
			retVal += pcBuff[ii] - 48;
		}
		else
		{
			// Error! Non-numeric number encountered.
			return 0;
		}
		retVal <<= 8;
	}
	return retVal;
}

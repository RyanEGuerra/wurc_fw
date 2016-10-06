/*
 * WSDFirm_main.c
 *   Main firmware for the Stellaris LM3S5Y36 Microcontroller on the WSD board. Enables
 *   SSI interfaces, USB interface, UART (for debug), and ADC subroutines for radio
 *   control.
 *
 *   This software is licensed to Rice University where applicable, Texas Instruments
 *   where applicable and should always carry this header description. No warranty is
 *   provided with this code, nor promise of suitability for any given purpose. Do not
 *   blame us if it breaks something.
 *
 *   You may not reuse this code without permission of the authors unless superseded
 *   by previous licensing. Isn't this stuff complicated? We should hire a code lawyer.
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 *   CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Created: Feb 26, 2012
 *  Edited:	July 27, 2013
 * 	Author: Ryan E. Guerra (me@ryaneguerra.com)
 * 	Author: Naren Anand (nanand@rice.edu)
 * 	Thanks: Holly Liang
 *
 *  Initial calibration algorithm work is thanks to Narendra, and a big thanks to Holly for
 *  initial testing with the Lime Dev Board.
 */


#include "include/wsd_main.h"

// After USB initialization, this contains a pointer to the USB CDC Device driver
// instance. This is used to specify the particular USB device in later driver calls.
void *pUSBDevice;

// Character/Command Buffers for the UART and USB interfaces, respectively.
// Both will process and print results to their respective interfaces, while
// keeping input from one separate from input from the other.
TerminalBuff_t sUSB0Buff;
TerminalBuff_t sUART0Buff;

//// Because dual-terminal functionality was added later, the main code
//// is not aware that there are two interfaces to print stuff to. This
//// pointer lets us register which terminal is the current active terminal.
//// When a command is executed, the originating terminal is registered here;
//// since all commands are "REST-ful," this means that the correct terminal
//// is always registered when output needs to be printed. Natch!
//TerminalBuff_t *g_activeTermBuff;

// Use for toggling debug state. Remove in production //FIXME
char g_cTxRxSwitchState = 0;

// Some stuff required for debugging libraries or some such nonsense.
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// The UART0 interrupt handler.
// Signal to the main program loop that characters are in the UART buffer!
//
//*****************************************************************************
void
UART0IntHandler(void)
{
	unsigned long ulStatus;

	// Get the interrupt status.
	ulStatus = ROM_UARTIntStatus(UART0_BASE, true);
	// Clear the asserted interrupts.
	ROM_UARTIntClear(UART0_BASE, ulStatus);

#ifdef __ENABLE_UART__
	// signal to the main loop that the UART queue is full
	iUART0BufferHasChars = 1;
#endif //__ENABLE_UART__

	//enable RX interrupts again
//    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX);
}

//*****************************************************************************
//
// Setup and enable the UART0 Peripheral. Enables the use of UARTprintf()
//
//*****************************************************************************
void
SetupUART0(void)
{
	// Enable UART0 peripheral and GPIOA
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Note that GPIOA is already enabled in the main code, but I'm leaving
	// this here so that you won't forget that it's required to do anything.
	//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Set GPIO A0 and A1 as UART pins.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));
	ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

	// Configure & enable the UART interrupts
	ROM_UARTIntClear(UART0_BASE, ROM_UARTIntStatus(UART0_BASE, false));
	ROM_UARTIntEnable(UART0_BASE, (UART_INT_OE | UART_INT_BE | UART_INT_PE |
									   UART_INT_FE | UART_INT_RT | UART_INT_RX));
}

#ifdef __ENABLE_USB__
	//*****************************************************************************
	//
	// Setup and enable the USB0 Peripheral.
	// This is set up as a USB Communications Device Class device and we use the
	// StellarisWare CDC Serial Device Driver, info on pg. 80 SW-USBL-UG-5228
	//
	//*****************************************************************************
	void
	SetupUSB0(void)
	{
		// Initialize the transmit and receive buffers.
		USBBufferInit(&g_sTxBuffer);
		USBBufferInit(&g_sRxBuffer);
		// Set the USB stack mode to Device mode with VBUS monitoring.
		USBStackModeSet(0, USB_MODE_DEVICE, 0);
		// Pass the device information to the USB library and place the device
		// on the bus. This initializes USB CDC device controller 0 with the descriptor
		// containing the two buffers we just initialized. Returns a CDC dev pointer or NULL
		// on failure to initialize.
		pUSBDevice = USBDCDCInit(0, &g_sCDCDevice);
	}
#endif //__ENABLE_USB__

//*****************************************************************************
//
// Setup and enable the SSI0 Peripheral.
// This is set up as a USB Communications Device Class device and we use the
// StellarisWare CDC Serial Device Driver, info on pg. 80 SW-USBL-UG-5228
//
//*****************************************************************************
void
SetupSSI0(void)
{
	// Select the correct function for the following SSI pins
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

	// Enable the peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	// Set the SSI bit clock to the slowest possible
	//TODO - we want this to default to the fastest, but I've been seeing a lot of
	//       read/write errors that I'm not happy with.
	Stel_SetSSI0ClkFast();
}

//*****************************************************************************
//
// ISR for Port B. Used for FPGA communications. Updates Soft Tx/Rx settings.
//
// Note that the pin to interrupt on depends on the WAB revision:
//   WAB Revision 1 interrupts on pins 0, 1
//   WAB Revision 2 interrupts on pins 6, 7 (NMI)
//
//*****************************************************************************
void
GPIO_PortB_IntHandler(void)
{
	unsigned long ulTXPinState;

	// Clear interrupts for both pins
	ROM_GPIOPinIntClear(CTRL_FPGA_BASE, CTRL_FPGA_NMI_PIN | CTRL_FPGA_TXEN_PIN );

	// Read current pin state
	ulTXPinState = ROM_GPIOPinRead(CTRL_FPGA_BASE, CTRL_FPGA_TXEN_PIN);

	// Set Tx or Rx mode accordingly.
	if (ulTXPinState)
	{
		// SET TO TRANSMIT
		if (trx.TxRxSwitchingMode == TxRxSWMode_TDD)
		{
			// TDD-mode switching
			__LMS_SET_TX_TDD;
		}
		else
		{
			// FDD-mode switching
			__LMS_SET_TX_FDD;
		}
	}
	else
	{
		// SET TO RECEIVE
		if (trx.TxRxSwitchingMode == TxRxSWMode_TDD)
		{
			// TDD-mode switching
			__LMS_SET_RX_TDD;
		}
		else
		{
			// FDD-mode switching
			__LMS_SET_RX_FDD;
		}
	}
}

//*****************************************************************************
//
// Setup and configure the GPIO lines connected to the FPGA. This function
// currently just handles the TX_SW_En pin and the NMI pin. We don't really
// have a use for the NMI pin at the moment.
//
//*****************************************************************************
void
SetupFPGA_GPIO(void)
{
	// Set as input
	ROM_GPIOPinTypeGPIOInput(CTRL_FPGA_BASE,
			 	 	 	     CTRL_FPGA_TXEN_PIN | CTRL_FPGA_NMI_PIN);

	// Set to interrupt on both rising and falling edge
	ROM_GPIOIntTypeSet(CTRL_FPGA_BASE,
					   CTRL_FPGA_TXEN_PIN | CTRL_FPGA_NMI_PIN,
					   GPIO_BOTH_EDGES);

	// Enable the pin interrupts
	ROM_GPIOPinIntEnable(CTRL_FPGA_BASE,
						 CTRL_FPGA_TXEN_PIN | CTRL_FPGA_NMI_PIN);
}

//*****************************************************************************
//
// Setup and enable the LED indicator on pin PA7. The LED is active-low.
// The LED circuit is set to require 5mA current.
//
//*****************************************************************************
void
SetupLEDs(void)
{
	unsigned long ulLEDPins = GPIO_PIN_7 | GPIO_PIN_6;
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, ulLEDPins);
	// standard push-pull pin driven at 8mA. Note only a few pins can be 8mA drive.
	GPIOPadConfigSet(GPIO_PORTA_BASE, ulLEDPins, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

	// Default configurations
	Stel_TurnOffErrorLED();
	Stel_TurnOffHeartbeatLED();
}

//*****************************************************************************
//
// Setup and enable the RF control lines. They are all active-high and pulled-low.
// The pulldown resistor sinks 0.33mA across 10kOhm resistors, so a 2mA drive
// for control should be just fine.
//
//*****************************************************************************
void
SetupRFCtrl(void)
{
	// setup port C[4-7] to default to output low.
	unsigned long ulCtrlPinsC =
			(CTRL_UHF_ANT_PIN | CTRL_UHF_PA2_PIN | CTRL_WIFI_ANT_PIN | CTRL_WIFI_PA_PIN);
	GPIOPinWrite(GPIO_PORTC_BASE, ulCtrlPinsC, 0x00);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, ulCtrlPinsC);
	GPIOPadConfigSet(GPIO_PORTC_BASE, ulCtrlPinsC, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	// setup port E[4] to default to output low.
	GPIOPinWrite(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN, 0x00);
	GPIOPinTypeGPIOOutput(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN);
	GPIOPadConfigSet(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	// setup port B[5] to default to output low.
	GPIOPinWrite(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN, 0x00);
	GPIOPinTypeGPIOOutput(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN);
	GPIOPadConfigSet(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

//*****************************************************************************
//
// This function should be called whenever a serial terminal is established
// between the ucontroller and some client terminal. Right now it prints a splash
// page and clears buffers. In the future, it may reset other things.
//
//*****************************************************************************
void
Stel_ResetTerminalDevice(void)
{
	// clear the received command buffers so that old session data doesn't get
	// in the way of new session data
	volo_resetCommandBuffer(&sUSB0Buff);
	g_activeTermBuff = &sUSB0Buff;

	// print splash screen - this may contain other information that the client uses
	// to determine what firmware is operating on this device.
	PrintStr("===============================================================\r\n");
	PrintStr("===============================================================\r\n");
	PrintStr("  __________-------____                 ____-------__________\r\n");
	PrintStr("  \\------____-------___--__---------__--___-------____------/\r\n");
	PrintStr("   \\//////// / / / / / \\   _-------_   / \\ \\ \\ \\ \\ \\\\\\\\\\\\\\\\/\r\n");
	PrintStr("     \\////-/-/------/_/_| /___   ___\\ |_\\_\\------\\-\\-\\\\\\\\/\r\n");
	PrintStr("       --//// / /  /  //|| (O)\\ /(O) ||\\  \\  \\ \\ \\\\\\\\--\r\n");
	PrintStr("            ---__/  // /| \\_  /V\\  _/ |\\ \\  \\__---\r\n");
	PrintStr("                 -//  / /\\_ ------- _/\\ \\  \\\\-\r\n");
	PrintStr("                   \\_/_/ /\\---------/\\ \\_\\_/\r\n");
	PrintStr("                       ----\\   |   /----\r\n");
	PrintStr("                            | -|- |\r\n");
	PrintStr("                           /   |   \\\r\n");
	PrintStr("                           ---- \\___|\r\n");
	PrintStr("                    __      __   _                           \r\n");
	PrintStr("                    \\ \\    / /  | |                          \r\n");
	PrintStr("                     \\ \\  / /__ | | ___                      \r\n");
	PrintStr("                      \\ \\/ / _ \\| |/ _ \\                     \r\n");
	PrintStr("                       \\  / (_) | | (_) |                    \r\n");
	PrintStr("          __          __\\/ \\___/|_|\\___/                     \r\n");
	PrintStr("          \\ \\        / (_)        | |              \r\n");
	PrintStr("           \\ \\  /\\  / / _ _ __ ___| | ___  ___ ___ \r\n");
	PrintStr("            \\ \\/  \\/ / | | '__/ _ \\ |/ _ \\/ __/ __|\r\n");
	PrintStr("             \\  /\\  /  | | | |  __/ |  __/\\__ \\__ \\\r\n");
	PrintStr("              \\/  \\/   |_|_|  \\___|_|\\___||___/___/\r\n");
	PrintStr("===============================================================\r\n");
	PrintStr("===============================================================\r\n");
	PrintStr("                   WURC Firmware Version ");
	PrintStr(pucWSDFirmVersion);
	__newline
	PrintStr("               Last Compiled: ");
	PrintStr(pucWSDFirmRevDate);
	PrintStr(" at ");
	PrintStr(pucWSDFirmRevTime);
	__newline
	PrintStr("                        Volo Wireless, LLC\n\r");
	PrintStr("===============================================================\r\n");
	PrintStr("===============================================================\r\n");
//	PrintStr("===================================================================\r\n");
//	PrintStr("===================================================================\r\n");
//	PrintStr(" \r\n");
//	PrintStr(" \r\n");
//	PrintStr("__/\\\\\\________/\\\\\\________________/\\\\\\\\\\\\__________________        \r\n");
//	PrintStr(" _\\/\\\\\\_______\\/\\\\\\_______________\\////\\\\\\__________________       \r\n");
//	PrintStr("  _\\//\\\\\\______/\\\\\\___________________\\/\\\\\\__________________      \r\n");
//	PrintStr("   __\\//\\\\\\____/\\\\\\_______/\\\\\\\\\\_______\\/\\\\\\________/\\\\\\\\\\____     \r\n");
//	PrintStr("    ___\\//\\\\\\__/\\\\\\______/\\\\\\///\\\\\\_____\\/\\\\\\______/\\\\\\///\\\\\\__    \r\n");
//	PrintStr("     ____\\//\\\\\\/\\\\\\______/\\\\\\__\\//\\\\\\____\\/\\\\\\_____/\\\\\\__\\//\\\\\\_   \r\n");
//	PrintStr("      _____\\//\\\\\\\\\\______\\//\\\\\\__/\\\\\\_____\\/\\\\\\____\\//\\\\\\__/\\\\\\__  \r\n");
//	PrintStr("       ______\\//\\\\\\________\\///\\\\\\\\\\/____/\\\\\\\\\\\\\\\\\\__\\///\\\\\\\\\\/___ \r\n");
//	PrintStr("        _______\\///___________\\/////_____\\/////////_____\\/////_____\r\n");
//	PrintStr(" \r\n");
//	PrintStr(" \r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr("   \r\n");
//	PrintStr("      __ |  / /________  /_____                            \r\n");
//	PrintStr("      __ | / /_  __ \\_  /_  __ \\                           \r\n");
//	PrintStr("      __ |/ / / /_/ /  / / /_/ /                           \r\n");
//	PrintStr("      _____/  \\____//_/  \\____/                            \r\n");
//	PrintStr("                                                           \r\n");
//	PrintStr("      ___       ______            ______                   \r\n");
//	PrintStr("      __ |     / /__(_)______________  /___________________\r\n");
//	PrintStr("      __ | /| / /__  /__  ___/  _ \\_  /_  _ \\_  ___/_  ___/\r\n");
//	PrintStr("      __ |/ |/ / _  / _  /   /  __/  / /  __/(__  )_(__  ) \r\n");
//	PrintStr("      ____/|__/  /_/  /_/    \\___//_/  \\___//____/ /____/  \r\n");
//	PrintStr("  \r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr(" \r\n");
//	PrintStr(" \r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr(" \r\n");
//	PrintStr("         __      __   _                           \r\n");
//	PrintStr("         \\ \\    / /  | |                          \r\n");
//	PrintStr("          \\ \\  / /__ | | ___                      \r\n");
//	PrintStr("           \\ \\/ / _ \\| |/ _ \\                     \r\n");
//	PrintStr("            \\  / (_) | | (_) |                    \r\n");
//	PrintStr("         __  \\/ \\___/|_|\\___/     _               \r\n");
//	PrintStr("         \\ \\        / (_)        | |              \r\n");
//	PrintStr("          \\ \\  /\\  / / _ _ __ ___| | ___  ___ ___ \r\n");
//	PrintStr("           \\ \\/  \\/ / | | '__/ _ \\ |/ _ \\/ __/ __|\r\n");
//	PrintStr("            \\  /\\  /  | | | |  __/ |  __/\\__ \\__ \\\r\n");
//	PrintStr("             \\/  \\/   |_|_|  \\___|_|\\___||___/___/\r\n");
//	PrintStr(" \r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr(" \r\n");
//	PrintStr("         _    __      __                       \r\n");
//	PrintStr("        | |  / /___  / /___                    \r\n");
//	PrintStr("        | | / / __ \\/ / __ \\                   \r\n");
//	PrintStr("        | |/ / /_/ / / /_/ /                   \r\n");
//	PrintStr("        |___/\\______/\\____/    __              \r\n");
//	PrintStr("        | |     / (_)_______  / /__  __________\r\n");
//	PrintStr("        | | /| / / / ___/ _ \\/ / _ \\/ ___/ ___/\r\n");
//	PrintStr("        | |/ |/ / / /  /  __/ /  __(__  |__  ) \r\n");
//	PrintStr("        |__/|__/_/_/   \\___/_/\\___/____/____/  \r\n");
//	PrintStr(" \r\n");
//	PrintStr("===============================================================\r\n");
//	PrintStr("===============================================================\r\n");

	// Reload the calibration values -- this should
	if (Stel_InitFlashCalibrationTables())
	{
		PrintStr("Warning: RF Calibration Table is Uninitialized");
		__newline
	}

	PrintStr("           Press \"h\" + RETURN to read help menu.");
	__newline

	// finally, print the terminal prompt so that the client or client app knows we're ready
	PrintStr("wss$ ");

#ifdef __DEBUG_UART__
	PrintStrUARTDebug("Done resetting\r\n");
#endif
}

//*****************************************************************************
//
// Clears global command buffers w/ memset.
//
//*****************************************************************************
//void
//ClearCommandBuffers(void)
//{
//	memset(pcOpCodeBuff, 0, sizeof(char)*OPCODE_MAX_SIZE);
//	memset(pcOperandBuff, 0, sizeof(char)*OPERAND_MAX_SIZE);
//	iNumCharsInCommandBuffer = 0;
//
//	return;
//}

////*****************************************************************************
////
//// The ltoa function is also somewhat under-whelming. This is a lazy hack.
//// Converts UPPER case HEX strings to a unsigned long.
////TODO improve or remove this ltoa hack.
////
//// \param cBuff The string to convert to an unsigned long value. MUST be in HEX
////              format. This function assumes the string is in upper-case HEX.
//// \param iLength The exact length of the string to convert. If the string is
////                longer, we will only try to convert the first iLength chars.
////
//// \return An unsigned long containing the converted data value.
////
////*****************************************************************************
//unsigned long
//myHex2Long(char *cBuff, int iLength)
//{
//	unsigned long ulReturn = 0;
//	unsigned long ulTemp;
//	int index;
//
//	// Rationality check
//	if (iLength <= 0)
//	{
//		Stel_ThrowError("16");
//		return 0;
//	}
//
//	// until we hit the end of this string...
//	for (index = iLength - 1; index >= 0; index--)
//	{
//		ulTemp = (unsigned long) cBuff[index];
//		// ASCII 0 is 48, ASCII 9 is 57
//		// ASCII A is 65, ASCII F is 70
//		if (ulTemp < 48 || ulTemp > 70)
//		{
//			Stel_ThrowError("Non-numeric Char");
//			return 0;
//		}
//		// value is A-F
//		if (ulTemp > 57)
//		{
//			ulTemp -= 55;
//		}
//		// value is 0-9
//		else
//		{
//			ulTemp -= 48;
//		}
//		// a little complex, just shift the temp variable to the right bit
//		// location and add the value to the running sum.
//		ulReturn = ulReturn + (ulTemp << ((iLength-1-index)*4));
//	}
//
//	return ulReturn;
//}

////*****************************************************************************
////
////TODO - bounds checking, also make sure it's actually a number
////
////*****************************************************************************
//unsigned long
//myDec2Long(char *cBuff)
//{
//	const int iLen = strlen(cBuff);
//	int jj;
//	unsigned long ulBuff;
//	unsigned long ulIndex, ulPow, ulCoEff;
////	char cDebugBuff[10];
//
//	// Rationality check
//	if (iLen <= 0)
//	{
//		Stel_ThrowError("17");
//		return 0;
//	}
//
//	// Get the LSD
//	ulBuff = myHex2Long(cBuff+iLen-1, 1);
//	for (ulIndex = 0; ulIndex < iLen-1; ulIndex++)
//	{
//		// 1, 2, 3, etc... not 0; that's handled above
//		ulPow = (unsigned long)iLen - ulIndex -  (unsigned long)1;
//		ulCoEff = 10;
//		for (jj = 1; jj < ulPow; jj++)
//		{
//			ulCoEff *= 10;
//		}
//		// Starting from the MSD, keep adding in the contribution
//		ulBuff += ulCoEff * myHex2Long(cBuff+ulIndex, 1);
//
////		PrintStr("> ");
////		myLong2Hex(ulBuff, cDebugBuff, 1);
////		PrintStr((unsigned char *)cDebugBuff);
////		__newline
//	}
//
//	return ulBuff;
//}

////*****************************************************************************
////
//// I'm kinda annoyed, but the built-in atol doesn't seem to handle the number
//// zero properly. It also seems to SUCK. So this catches that case. WTF.
//// I'm a terrible coder and this is probably the worst way to do this.
//// TI, I am disappoint.
////
//// \param ulVal The unsigned long value to convert to a hex string.
//// \param cBuff A character buffer to placed the formatted hex string into.
//// \param iLength The minimum length of the string to print out. Note this is
////                ONLY a minimum! Make sure the buffer is large enough.
////
////*****************************************************************************
//void
//myLong2Hex(unsigned long ulVal, char *cBuff, int iLength)
//{
//	unsigned long ulTest;
//	int jj;
//	int offset = 0;
//
//	for(jj = 0; jj < 8; jj++)
//	{
//		// mask out the top 4 bits, then bit shift to get values [0-F]
//		ulTest = (ulVal & 0xF0000000) >> 28;
//
//		// whenever we've already printed the non-zero MSD or we're hitting
//		// the first non-zero MSD, we print a character. We also force a
//		// character to be printed if this is the LOWEST 4 bits since we
//		// have a valid char output when ulVal is zero
//		if (offset > 1 || ulTest > 0 || jj >= (8-iLength))
//		{
//			if (ulTest < 10)
//			{
//				// 48 is the ASCII offset for [0-9]
//				cBuff[offset] = (char) (ulTest + 48);
//			}
//			else
//			{
//				// ASCII A = 65; A = 10; so ASCII offset for [A-F] is 55
//				cBuff[offset] = (char) (ulTest + 55);
//			}
//			// move on to the next character
//			offset++;
//		}
//
//		// check the next 4 bits
//		ulVal = ulVal << 4;
//	}
//	// don't forget the null terminator!
//	cBuff[offset] = '\0';
//
//	return;
//}

////*****************************************************************************
////
//// Prints a given number of characters from the passed char* to a given
//// interface. This helper function is required to make code interface-
//// neutral. Relies on preprocessor variables to determine which interface
//// is used.
////
//// \param ucStr The string to print.
//// \param ulCharsToWrite The number of characters to actually print.
////
//// \return The number of chars actually written to the interface.
////
////*****************************************************************************
//unsigned long
//Print(const unsigned char *pucStr, unsigned long ulCharsToWrite)
//{
//	unsigned long ulNumWritten = 0;
//
//#ifdef __ENABLE_USB__
//	ulNumWritten = PrintUSB(pucStr, ulCharsToWrite);
//#endif //__ENABLE_USB
//
//#ifdef __ENABLE_UART__
//	ulNumWritten = PrintUART(pucStr, ulCharsToWrite);
//#endif //ENABLE_UART
//
//	return ulNumWritten;
//}

#ifdef __ENABLE_USB__
////*****************************************************************************
////
//// Prints a given number of characters from the passed char* to the USB0 iface.
//// Print ulCharsToWrite number of chars or until the end of the string, whichever
//// is first.
//// Note that this function currently blocks if the USB0 TX buffer is full. This
//// is not safe, but fixing it is annoying.
////
//// \param ucStr The string to print.
//// \param ulCharsToWrite The number of characters to actually print.
////
//// \return The number of chars actually written to the USB0 buffer.
////
////*****************************************************************************
//unsigned long
//PrintUSB(const unsigned char *pucStr, unsigned long ulCharsToWrite)
//{
//	unsigned long ulCharsWritten = 0;
//	unsigned long ulSpace;
//	unsigned long usbTimeoutCount = 0;
//	char usbIsStuck = FALSE;
//
//	ulSpace = USBBufferSpaceAvailable(&g_sTxBuffer);
//
//	// Read data from the string buffer until there is none left or there is no
//	// more space in the USB TX buffer, and add them to the USB TX buffer.
//	// Also exit if we've reached the end of the string!
//	while((ulCharsToWrite - ulCharsWritten > 0) && (pucStr[ulCharsWritten] != '\0'))
//	{
//		// block until space opens up.
//		//todo - come back and optimize this
//
//		while (ulSpace == 0)
//		{
//			ulSpace = USBBufferSpaceAvailable(&g_sTxBuffer);
//			// TODO - make this safe--we tried, but it's a bit buggy and we
//			// fixed the problem in other ways
//			if (++usbTimeoutCount > USB_TIMEOUT_MAX_COUNT)
//			{
//				usbIsStuck = TRUE;
//				break;
//			}
//		}
//
//		//Reset the timeout, since we made it outs
//		usbTimeoutCount = 0;
//
//		if (usbIsStuck)
//		{
//			// TODO - come back and fix properly
//			ulCharsWritten = 10;
//			break;
//		}
//
//		// get the next char in the buffer and write it
//		USBBufferWrite(&g_sTxBuffer, pucStr + ulCharsWritten, 1);
//		ulCharsWritten++;
//
//		// check the remaining space available to the USB buffer
//		ulSpace = USBBufferSpaceAvailable(&g_sTxBuffer);
//	}
//
//	return ulCharsWritten;
//}
#endif //__DEFINE_USB__

////*****************************************************************************
////
//// Prints a given number of characters from the passed char* to the UART0 iface.
////
//// \param ucStr The string to print.
//// \param ulCharsToWrite The number of characters to actually print.
////
//// \return The number of chars actually written to the USB0 buffer.
////
////*****************************************************************************
//unsigned long
//PrintUART(const unsigned char *pucStr, unsigned long ulCharsToWrite)
//{
//	unsigned long ulCharsWritten = 0;
//
//		// While there are more Chars to write or until the string ends
//		while((ulCharsToWrite - ulCharsWritten > 0) && (pucStr[ulCharsWritten] != '\0'))
//		{
//			// If there is space, write more chars to the buffer!
//			//TODO - this is not safe; check to see if there is another way
//			if (ROM_UARTSpaceAvail(UART0_BASE))
//			{
//				ROM_UARTCharPutNonBlocking(UART0_BASE, pucStr[ulCharsWritten]);
//				ulCharsWritten++;
//			}
//		}
//
//	return ulCharsWritten;
//}

////*****************************************************************************
////
//// Prints the entirety of the passed char* to the currently active iface.
//// Note that USB0print currently blocks if the USB0 TX buffer is full. This
//// is not safe, but fixing it is annoying.
////
//// \param ucStr The string to print.
////
////*****************************************************************************
//void
//PrintStr(const unsigned char *pucStr)
//{
//	__terminalPrint(g_activeTermBuff, (char *)pucStr);
//
//	return;
//}

////*****************************************************************************
//// Main error handling function; wraps the error msg in "!" chars for external
//// processing and also turns the error LED to active.
////
//// \param The error description string to print to UART.
////*****************************************************************************
//void
//Stel_ThrowError(const unsigned char *pucStr)
//{
//	PrintStr("! ");
//	PrintStr(pucStr);
//	PrintStr(" !");
//	__newline
//
//	Stel_TurnOnErrorLED();
//}

////*****************************************************************************
//// Stripped down USB print function that accepts a null-terminated string.
//// Rather than some other functions that let us specify the length of
//// the string to print.
////
//// \param pucStr The null-terminated string to print to the USB interface.
////*****************************************************************************
//void
//PrintStrUSB(char *pucStr)
//{
//	unsigned long ulStrLen;
//
//	ulStrLen = strlen(pucStr);
//	PrintUSB((unsigned char*)pucStr, ulStrLen);
//}

////*****************************************************************************
//// Stripped down UART print function that accepts a null-terminated string.
//// Rather than some other functions that let us specify the length of
//// the string to print.
////
//// \param pucStr The null-terminated string to print to the UART interface.
////*****************************************************************************
//void
//PrintStrUART(char *pucStr)
//{
//	unsigned long ulStrLen;
//
//	ulStrLen = strlen(pucStr);
//	PrintUART((unsigned char*)pucStr, ulStrLen);
//}

//*****************************************************************************
//
// ONLY USE THIS IF YOU KNOW WHAT YOU'RE DOING! Rather than dynamically choose
// the "correct" terminal to print output to, this function just ALWAYS sends
// it via UART to the BB host. IF this is what you want, sobeit.
//
// \param ucStr The string to print.
//
// \return The number of chars actually written to the UART0 buffer.
//
//*****************************************************************************
unsigned long
PrintStrUARTDebug(const unsigned char *pucStr)
{
	unsigned long ulCharsToWrite;
	unsigned long ulCharsWritten;

	// this dumb cast is required because strlen take char* and USBBufferWrite
	// takes unsigned char* - removes warning.
	ulCharsToWrite = strlen((char *)pucStr);

	ulCharsWritten = PrintUART(pucStr, ulCharsToWrite);

	return ulCharsWritten;
}

#ifdef __ENABLE_USB__
//*****************************************************************************
//
// DO NOT USE OUTSIDE OF DEBUG!
//
// \param ucStr The string to print.
//
// \return The number of chars actually written to the USB0 buffer.
//
//*****************************************************************************
unsigned long
PrintStrUSBDebug(const unsigned char *pucStr)
{
	unsigned long ulCharsToWrite;
	unsigned long ulCharsWritten;

	// this dumb cast is required because strlen take char* and USBBufferWrite
	// takes unsigned char* - removes warning.
	ulCharsToWrite = strlen((char *)pucStr);

	ulCharsWritten = PrintUSB(pucStr, ulCharsToWrite);

	return ulCharsWritten;
}
#endif //__ENABLE_USB__

////*****************************************************************************
////
//// Test function to echo back anything typed in the UART0;
////
////*****************************************************************************
//void
//EchoUART(void)
//{
//	unsigned long ulBuff;
//
//    // Loop while there are characters in the receive FIFO.
//    while(ROM_UARTCharsAvail(UART0_BASE))
//    {
//        // Read the next character from the UART and write it back to the UART.
//    	ulBuff =  ROM_UARTCharGetNonBlocking(UART0_BASE);
//        ROM_UARTCharPutNonBlocking(UART0_BASE, ulBuff & 0xFF);
//    }
//}

//*****************************************************************************
//
// Main input processor for the USB0 peripheral. This will pull characters from
// the USB0 buffer until there are none left, adding them to the terminal
// buffer and executing commands, if necessary.
//
//*****************************************************************************
void
Stel_ProcessUSB0Input()
{
	unsigned char ucChar;

	// Pull and process chars from the USB0 interface until there are no more
	// chars to pull.
	do
	{
		// Try to read up to 1 character. The return value is the number actually
		// in the USB RX queue.
		if (USBBufferRead(&g_sRxBuffer, &ucChar, 1) == 0)
		{
			// No chars available, so return a null char.
			ucChar = '\0';
		}
		else
		{
			// Add the retrieved char to the terminal buffer & process.
			volo_processChar(&sUSB0Buff, (char)ucChar);
		}
	} while (ucChar != '\0');

	// Buffer empty; done!
	return;
}

//*****************************************************************************
//
// Main input processor for the UART0 peripheral. This will pull characters from
// the UART0 buffer until there are none left, adding them to the terminal
// buffer and executing commands, if necessary.
//
//*****************************************************************************
void
Stel_ProcessUART0Input()
{
	unsigned char ucChar;

	// Pull and process chars from the UART0 interface until there are
	// no more chars to pull.
	do
	{
		// Check to see if any more chars are available in the UART RX buffer.
		if (ROM_UARTCharsAvail(UART0_BASE))
		{
			ucChar = ROM_UARTCharGetNonBlocking(UART0_BASE);
			// Add the retrieved char to the terminal buffer & process.
			volo_processChar(&sUART0Buff, (char)ucChar);
		}
		else
		{
			// No chars available, so return a null char.
			ucChar = '\0';
		}
	} while (ucChar != '\0');

	// Buffer empty; done!
	return;
}

//*****************************************************************************
//
// Replacement command-processing function that can handle commands from either
// USB or UART. This is decared extern from volo_term.h
//
// \param pBuff A terminal struct pointer containing command/opcode buffs
//              & callback funcs
//
//*****************************************************************************
void
volo_executeCommand(TerminalBuff_t* pBuff)
{
	// Since the command-processing code is copy/pasted from the old term
	// processing function, we declare local symbols for compatibility.
	char ucOpCode = pBuff->cOpCodeBuff[0];
	char *pcOperand = pBuff->cCmdBuff;

	// Register the terminal buffer that originated this command so that all
	// responses can be printed to it.
	g_activeTermBuff = pBuff;

	// This is required to make things print nicely. Otherwise, it gets a bit ugly.
	__newline

	// Process the passed command.
	switch(ucOpCode)
	{
	// Read a register value from the Lime Xceiver.
	// Example command: r 3A
	// ================================================================================
		case OP_READ_REG:
		{
			unsigned long ulReadAddr;
			unsigned long ulReadData;
			char pcReadData[4];

			// check to make sure that operand is the right format
			if(strlen(pcOperand) != 2)
			{
				Stel_ThrowError("Operand is not right. Expecting length 2");
				break;
			}

			// convert operand char* into a ul
			ulReadAddr = myHex2Long(pcOperand, 2);
			Lime_Read(ulReadAddr, &ulReadData);
			myLong2Hex(ulReadData, pcReadData, 2);

			// output result to the USB0 interface.
			PrintStr((unsigned char *)pcReadData);
			__newline
			break;
		}

	// Write to the Lime Xceiver with robust readback & error checking
	// Example command: w 5F87
	// ================================================================================
		case OP_RWRITE_REG:
		{
			unsigned long ulWriteAddr;
			unsigned long ulWriteData;
			int iRetVal;
			char pcBuff[4];

			// check to make sure that operand is the right format
			if(strlen(pcOperand) != 4)
			{
				Stel_ThrowError("Operand is not right. Expecting length 4");
				break;
			}
			// try to convert strings into unsigned longs
			strncpy(pcBuff, pcOperand, 2);
			ulWriteAddr = myHex2Long(pcBuff, 2);
			strncpy(pcBuff, pcOperand + 2, 2);
			ulWriteData = myHex2Long(pcBuff, 2);

			// write the data and check the returned error flags
			iRetVal = Lime_RobustWrite(ulWriteAddr, ulWriteData);
			myLong2Hex((unsigned long)iRetVal, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			__newline
			break;
		}

	// Write to the target register and suppress additional output
	// Example command: v 5F87
	// ================================================================================
		case OP_SILENT_WRITE_REG:
		{
			unsigned long ulWriteAddr;
			unsigned long ulWriteData;
			char pcBuff[4];
			int errors = 0;

			// try to convert strings into unsigned longs
			strncpy(pcBuff, pcOperand, 2);
			ulWriteAddr = myHex2Long(pcBuff, 2);
			strncpy(pcBuff, pcOperand + 2, 2);
			ulWriteData = myHex2Long(pcBuff, 2);

			// write the data and check the returned error flags
			errors = Lime_RobustWrite(ulWriteAddr, ulWriteData);
			myLong2Hex((unsigned long)errors, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			__newline
			break;
		}

	// Clear error LED. Clears the error LED so that if it turns on, that means something.
	// Example: e
	// ================================================================================
		case OP_CLEAR_ERROR:
		{
			Stel_TurnOffErrorLED();
			break;
		}

	// Execute the Auto Calibration Sequence sec 4.7, pg. 41 in the LMS6002D
	// Programming & Calibration Guide. Then set a number of default settings for the transceiver.
	// Example: A
	// ================================================================================
		case OP_PWR_ON_INIT:
		{
			PrintStr("DEBUG - CLOCK REG: ");
			Stel_PrintReg_DEBUG(0x09);
			__newline

			if (Lime_XceiverPwrOnInitializationSequence())
			{
				// I found that this occurs when the FPGA is not supplying
				// a reference clock. Print this message to remind people.
				Stel_ThrowError("       Error on Initialization      ");
				PrintStr("CHECK REFERENCE CLOCKS");
				__newline
			}
			break;
		}

	// Setup for TX
	// Example: N
	// ================================================================================
		case OP_SET_TX:
		{
			if (trx.TxRxSwitchingMode == TxRxSWMode_TDD)
			{
				__LMS_SET_TX_TDD;
			}
			else
			{
				__LMS_SET_TX_FDD;
			}

			PrintStr("TX on ANT1");
			__newline
			break;
		}

	// Setup for RX
	// Example: M
	// ================================================================================
		case OP_SET_RX:
		{
			if (trx.TxRxSwitchingMode == TxRxSWMode_TDD)
			{
				__LMS_SET_TX_TDD;
			}
			else
			{
				__LMS_SET_RX_FDD;
			}

			PrintStr("RX on ANT1");
			__newline
			break;
		}

	// Set the TX center frequency. Operand is in kHz
	// Example: B 500000
	// ================================================================================
		case OP_SET_RX_FREQUENCY:
		{
			unsigned long ulTargetFrequencyKHz;
			char cBuff[10];

			ulTargetFrequencyKHz = myDec2Long(pcOperand);
			PrintStr("Setting RX Channel to HEX ");
			myLong2Hex(ulTargetFrequencyKHz, cBuff, 1);
			PrintStr((unsigned char *)cBuff);
			__newline

			// Try setting the frequency
			if (Lime_SetFrequency(LMS_RXPLL_BASE, ulTargetFrequencyKHz))
			{
				Stel_ThrowError("Set RxFreq Failed");
			}
			break;
		}

	// Set the RX center frequency. Operand is in kHz
	// Example: D 2400000
	// ================================================================================
		case OP_SET_TX_FREQUENCY:
		{
			unsigned long ulTargetFrequencyKHz;
			char cBuff[10];

			ulTargetFrequencyKHz = myDec2Long(pcOperand);
			PrintStr("Setting TX Channel to HEX ");
			myLong2Hex(ulTargetFrequencyKHz, cBuff, 1);
			PrintStr((unsigned char *)cBuff);
			__newline

			// Try setting the frequency
			if (Lime_SetFrequency(LMS_TXPLL_BASE, ulTargetFrequencyKHz))
			{
				Stel_ThrowError("Set Rx Freq Failed");
			}
			//Lime_AutoCalibrationSequence();
			break;
		}

	// Set the TX center frequency. Operand is in kHz
	// Example: B 500000
	// ================================================================================
		case OP_SET_CHANNEL:
		{
			unsigned long ulChanType, ulChan, ulChanBandwidth;
			char pcBuff[10];

			// check to make sure that operand is the right format
			if(strlen(pcOperand) != 5)
			{
				Stel_ThrowError("Bad op; exp len 5");
				break;
			}

			// try to convert strings into unsigned longs
			strncpy(pcBuff, pcOperand + 1, 2);
			ulChan = myDec2Long(pcBuff);
			strncpy(pcBuff, pcOperand + 3, 2);
			ulChanBandwidth = myDec2Long(pcBuff);

			// Decide what to do based on the channel type
			switch(pcOperand[0])
			{
				// US & Americas UHF Channel Type
				case 'A':
				{
					ulChanType = LMS_US_UHF;
					break;
				}
				// Europe/UK Channel Type
				case 'E':
				{
					ulChanType = LMS_EU_UHF;
					break;
				}
				// 802.11g Channel Type
				case 'W':
				{
					ulChanType = LMS_WIFI;
					break;
				}
				default:
				{
					Stel_ThrowError("Bad Chan Type");
					return;
				}
			}

			if (Lime_SetChannel(ulChanType, ulChan, ulChanBandwidth)) {
				Stel_ThrowError("Chan Set Err");
			}

			break;
		}

	// Toggle the error ind LED. This is great for testing if communications
	// are working and/or figuring out which physical board you're connected to.
	// Example: l
	// ================================================================================
		case OP_TOGGLE_LED:
		{
			unsigned long ulVal;

			// Read which value the error LED currently has
			ulVal = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6);
			if (ulVal)
			{
				// the LED is active-low
				Stel_TurnOnErrorLED();
			}
			else
			{
				Stel_TurnOffErrorLED();
			}
			break;
		}

	// Get information about the micro-controller and the attached LMS6002D.
	// Should display that information nicely.
	// Example: i
	// ================================================================================
		case OP_GETCHIPINFO:
		{
			unsigned long ulSerialNo = WSD_GetSerialNumber();
			char pcBuff[10];

			PrintStr("\r\nCommand \"i\" is deprecated. Use \"j\" instead.");
			// NOTE: the Python wrapper looks for "Serial" and a hex string
			// of arbitrary length on the same line. Do not distrupt that.
			__newline
			PrintStr("WSD_Hex_Serial: #");
			myLong2Hex(ulSerialNo, pcBuff, 5);
			PrintStr((unsigned char *)pcBuff);
			__newline
//			CalTable_t * pCalTable;
//
//			// chip information is stored in CPUID and DID0
//			// respectively, this is ARM version information
//			// and Stellaris chip version information
//			unsigned long ulCPUID = NVIC_CPUID_R;
//			unsigned long ulDID0 = SYSCTL_DID0_R;
//			unsigned long ulARMVariant = (ulCPUID & NVIC_CPUID_VAR_M) >> 20;
//			unsigned long ulARMRevision = (ulCPUID & NVIC_CPUID_REV_M);
//			unsigned long ulARMPartNo = (ulCPUID & NVIC_CPUID_PARTNO_M) >> 4;
//			unsigned long ulStellarisClass = (ulDID0 & SYSCTL_DID0_CLASS_M) >> 16;
//			unsigned long ulStellarisMajorRev = (ulDID0 & SYSCTL_DID0_MAJ_M) >> 8;
//			unsigned long ulStellarisMinorRev = (ulDID0 & SYSCTL_DID0_MIN_M);
//			unsigned long ulSerialNo = WSD_GetSerialNumber();
//			char pcBuff[9];
//
//			// We expect the following:
//			//			--> DID0: 0x10030200
//			//			--> CPUID: 0x412FC230
//			//			--> Stellaris Class: 3
//			//			              Version: 2.0
//			//			    ARM Core  Part: C23
//			//			              Version: 2.0
//			//			--> LMS6002D Silicon Version: 2.2
//			//			--> Firmware Version:0.3
//
////			myltoh(ulDID0, pcBuff, 8);
////			PrintStr("--> DID0: 0x");
////			PrintStr((unsigned char *)pcBuff);
////			__newline
////			myltoh(ulCPUID, pcBuff, 8);
////			PrintStr("--> CPUID: 0x");
////			PrintStr((unsigned char *)pcBuff);
////			__newline
//
//			PrintStr("Stellaris Class: ");
//			myLong2Hex(ulStellarisClass, pcBuff, 1);
//			PrintStr((unsigned char *)pcBuff);
//			__newline
//			PrintStr("          Version: ");
//			myLong2Hex(ulStellarisMajorRev, pcBuff, 1);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr(".");
//			myLong2Hex(ulStellarisMinorRev, pcBuff, 1);
//			PrintStr((unsigned char *)pcBuff);
//			__newline
//			PrintStr("ARM Core  Part: ");
//			myLong2Hex(ulARMPartNo, pcBuff, 1);
//			PrintStr((unsigned char *)pcBuff);
//			__newline
//			PrintStr("          Version: ");
//			myLong2Hex(ulARMVariant, pcBuff, 1);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr(".");
//			myLong2Hex(ulARMRevision, pcBuff, 1);
//			PrintStr((unsigned char *)pcBuff);
//			__newline
//			if (Lime_GetChipInformation())
//			{
//				Stel_ThrowError("CHECK LMS6002D SPI CONNECTION");
//			}
//			PrintStr("Firmware Version: ");
//			PrintStr(pucWSDFirmVersion);
//			__newline
//			PrintStr("Compiled: ");
//			PrintStr(pucWSDFirmRevDate);
//			PrintStr(" ");
//			PrintStr(pucWSDFirmRevTime);
//			__newline
//			// Try to retrieve the current calibration table
//			pCalTable = Stel_GetCalibrationTable();
//			PrintStr("Calibration Table: ");
//			PrintStr(pCalTable == NULL ? (unsigned char*)"UNINITIALIZED !" : (unsigned char*)"ok");
//			__newline
//			PrintStr("WSD Serial #");
//			myLong2Hex(ulSerialNo, pcBuff, 5);
//			PrintStr((unsigned char *)pcBuff);
//			__newline
			break;
		}

	// Get information about the current settings for the RF section.
	// Should display that information as a JSON-formatted string
	// Example: j
	// ================================================================================
		case OP_GETCHIPINFO_JSON:
		{
//			CalTable_t * pCalTable;
			__newline
			Stel_PrintStatusJSON(pBuff);
			__newline
			break;
		}

	// Get information about the current settings for the RF section.
	// Should display that information nicely.
	// Example: g
	// ================================================================================
		case OP_GETBACKDOORDATA:
		{

			// Do nothing but send the serial number to the higher layer.
//			PrintStr("    \"wsd_hex_Serial\": \"#");
//			myLong2Hex(WSD_GetSerialNumber(), pcBuff, 5);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr("\"");

			// This will print the
			BB_SetHostSerialNumber(WSD_GetSerialNumber());

			break;
		}

	// Toggle the PA states for testing and current measuring purposes.
	// Starts with all-off, then WiFi ON, all off, then UHF ON...
	// Example: t
	// ================================================================================
//		case OP_TOGGLE_PA:
//		{
//			switch(g_cPASelectToggleState)
//			{
////				case 0:
////				{
////					WSD_TurnOnWiFiPAs();
////					PrintStr("--> WiFi PAs On\n\r");
////					break;
////				}
////				case 1:
////				{
////					WSD_TurnOffWiFiPAs();
////					PrintStr("--> WiFi PAs Off\n\r");
////					break;
////				}
//				case 0:
//				{
//					WSD_TurnOnUHFPAs();
//					PrintStr("--> UHF PAs On\n\r");
//					break;
//				}
//				case 1:
//				{
//					WSD_TurnOffUHFPAs();
//					PrintStr("--> UHF PAs Off");
//					__newline
//					break;
//				}
//				default:
//				{
//					Stel_ThrowError("Unrecognized Toggle State");
//				}
//			}
//			g_cPASelectToggleState++;
//			g_cPASelectToggleState %= 2;
//			break;
//		}

	// Enable/disable the automatic lookup and writing of stored calibration values
	// when radio parameters change. This is a way to confirm (by hand) the validity
	// of the calibration table. Debug only. Don't use under normal operation.
	// Example: u
	// ================================================================================
		case OP_TOGGLE_CALLOAD:
		{
			if (pcOperand[0] == '0')
			{
				// DISABLE
				PrintStr("DISabling Auto Calibration Loading");
				__newline
				trx.automaticCalLoadingEnabled = FALSE;
			}
			else if (pcOperand[0] == '1')
			{
				// ENABLE
				PrintStr("ENabling Auto Calibration Loading");
				__newline
				trx.automaticCalLoadingEnabled = TRUE;
			}
			else
			{
				// TOGGLE if neither a '1' nor '0' is the argument
				if (trx.automaticCalLoadingEnabled == TRUE)
				{
					PrintStr("DISabling Auto Calibration Loading");
					__newline
					trx.automaticCalLoadingEnabled = FALSE;
				}
				else
				{
					PrintStr("ENabling Auto Calibration Loading");
					__newline
					trx.automaticCalLoadingEnabled = TRUE;
				}
			}
			break;
		}
	// Toggle the RXOUTSW to enable BB output before the ADC.
	// Example: T
	// ================================================================================
		case OP_TOGGLE_RXOUTSW:
		{
			if (pcOperand[0] == '0')
			{
				// Open the RxOUT switch
				Lime_OpenRXOutSwitch();
			}
			else if (pcOperand[0] == '1')
			{
				// Close the RxOUT switch
				Lime_CloseRXOutSwitch();
			}
			else
			{
				// Toggle the TxOUT Switch
				Lime_ToggleRXOutSwitch();
			}
			break;
		}

	// Toggle the Software RXEnable bit
	// Example: R
	// ================================================================================
		case OP_TOGGLE_SRXEN:
		{
			Lime_ToggleSRXEN();
			break;
		}

	// Toggle the Software TXEnable bit
	// Example: S
	// ================================================================================
		case OP_TOGGLE_STXEN:
		{
			Lime_ToggleSTXEN();
			break;
		}

	// Execute the Auto Calibration Sequence sec 4.7, pg. 41 in the LMS6002D
	// Programming & Calibration Guide. This calibrates all TX/RX modules for DCO
	// removal.
	// Example: F
	// ================================================================================
		case OP_AUTOCAL_SEQUENCE:
		{
			if (Lime_AutoCalibrationSequence())
			{
				//error occured
				PrintStr("=== Re-running AutoCalibration ===");
				__newline
				if (Lime_AutoCalibrationSequence())
				{
					Stel_ThrowError("Autocal Seq Err [5]");
				}
			}
			break;
		}

	// Set TX channel bandwidth
	// Example: V <0-14>
	// ================================================================================
		case OP_SET_TX_BW:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if (ulVal > 15)
			{
				Stel_ThrowError("OOB TX BW");
				break;
			}
			Lime_SetFilterBandwidth(LMS_TXLPF_BASE, ulVal);

			PrintStr("--> Set TX BW: ");
			PrintStr((unsigned char *)pcOperand);
			__newline

			break;
		}

	// Set RX channel bandwidth
	// Example: W <0-14>
	// ================================================================================
		case OP_SET_RX_BW:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if (ulVal > 15)
			{
				Stel_ThrowError("OOB RX BW");
				break;
			}
			Lime_SetFilterBandwidth(LMS_RXLPF_BASE, ulVal);

			PrintStr("--> Set RX BW: ");
			PrintStr((unsigned char *)pcOperand);
			__newline

			break;
		}

	// TXVGA1 Gain Setting
	// Example: G <0-31>
	// ================================================================================
		case OP_SET_TX_VGA1:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if (ulVal > 31)
			{
				Stel_ThrowError("OOB TXVGA1 Gain");
				break;
			}
			Lime_SetTXVGA1Gain(ulVal);
			break;
		}

	// TXVGA2 Gain Setting
	// Example: H <0-25>
	// ================================================================================
		case OP_SET_TX_VGA2:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if (ulVal > 25)
			{
				Stel_ThrowError("OOB TXVGA2 Gain");
				break;
			}
			Lime_SetTXVGA2Gain(ulVal);
			break;
		}

	// RXVGA2 Gain Setting - 0-10 => 0-30 dBm
	// Example: K <0-10>
	// ================================================================================
		case OP_SET_RX_VGA2:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if (ulVal > 10)
			{
				Stel_ThrowError("OOB RXVGA2 Gain");
				break;
			}
			Lime_SetRXVGA2Gain(ulVal);
			break;
		}

	// RXVGA1 Gain Setting: [0, 120] =>  [0, 30]dBm
	// Example: J  <0:120>
	// ================================================================================
		case OP_SET_RX_VGA1:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if (ulVal > 120)
			{
				Stel_ThrowError("RX VGA1 OOB");
				break;
			}
			Lime_SetRXVGA1Gain(ulVal);
			break;
		}

	// LNA Gain Setting: [0, 1, 2] => [0, 3, 6] dB
	// Example: J  <0:2>
	// ================================================================================
		case OP_SET_RX_LNAGAIN:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if (ulVal > 2)
			{
				Stel_ThrowError("RX LNA OOB");
				break;
			}
			Lime_SetLNAGain(ulVal);
			break;
		}

	// Quickly toggles the Soft LMS6002D reset and turns off PAs.
	// Example: X
	// ================================================================================
		case OP_SOFT_RESET:
		{
			if (Lime_SoftReset_PAsOff())
			{
				// error on non-zero.
				Stel_ThrowError("SReset Failed");
			}

			PrintStr("Soft LMS6002D reset. TX PAs disabled.");
			__newline
			break;
		}
		case OP_EASTER:
		{
			PrintStr("<3 you, Rose");
			__newline
			PrintStr("Sorry I'm here coding this.");
			__newline
			PrintStr("I'll be home soon, I promise.");
			__newline
			break;
		}

	// Print the current RX DCO values for checking & calibration
	// Example: P
	// ================================================================================
		case OP_PRINT_LOFT_DCO_VALS:
		{
			WSD_PrintCurrentLOFT_DCOSettings();
			break;
		}

	// Load the saved RX DCO values from non-volatile Flash memory
	// Example: O
	// ================================================================================
		case OP_SET_SERIAL_CODE:
		{
			unsigned long ulSerialCode;

			// Test the operand length
			switch(strlen(pcOperand))
			{
				// Load a serial # to the user flash registers
				// User enters: OXXXXX, where X = serial number in upper-case HEX
				case 5:
				{
					ulSerialCode = myHex2Long(pcOperand, 5);
					if(WSD_SetSerialNumber(ulSerialCode))
					{
						Stel_ThrowError("LoadSerFailed");
					}
					break;
				}
				// Commit the loaded serial #
				case 0:
				{
					WSD_CommitSerialNumber();
					break;
				}
				default:
				{
					// Error - bad operand length.
					Stel_ThrowError("Bad arg len [10]");
				}
			}

			break;
		}

		// Disable the transmit and receive RF chains and just use the DAC/ADC
		// Example: Q
		// ================================================================================
//		case OP_START_BB_MODE:
//		{
//			//FIXME
//
//			// Close the RxOUT switch to connect the ADC to BB input pins.
//			Lime_CloseRXOutSwitch();
//			break;
//		}

	// Select the global transceiver state
	// 0 = Normal
	// 1 = Bypass Internal LNA
	// 2 = RF Loopback
	// 3 = BB Loopback
	// Example: Z #
	// ================================================================================
		case OP_SELECT_TRX_MODE:
		{
			//TODO this is actually a mode thing
			if (pcOperand[0] == '0')
			{
//				WSD_TurnOnUHFLNA();
				PrintStr("TRX Normal Mode");
				__newline
				trx.TRXState = RX_NORMAL;
				Lime_SetRxInputPath(INTERNAL_LNA);
			}
			else if (pcOperand[0] == '1')
			{
//				WSD_TurnOffUHFLNA();
				PrintStr("TRX Bypass Internal LNA Mode");
				__newline
				Lime_SetRxInputPath(EXTERNAL_LNA);
				trx.TRXState = RX_BYPASS_INT_LNA;
			}
			else if (pcOperand[0] == '2')
			{
				PrintStr("RF Loopback Mode");
				__newline
				Lime_StartRFLoopbackMode();
				trx.TRXState = RX_LOOPBACK_RF;
			}
			else if (pcOperand[0] == '3')
			{
				PrintStr("BB Loopback Mode");
				__newline
				Lime_StartBBLoopbackMode();
				trx.TRXState = RX_LOOPBACK_BB;
			}
			else
			{
				PrintStr("!Unknown Arg!");
				__newline
			}
			break;
		}
	// Toggle the Tx/Rx baseband control. This enables/disables GPIO interrupts
	// for the Stellaris
	// Example: x
	// ================================================================================
		case OP_TOGGLE_TXRX_CTRL:
		{
			if (pcOperand[0] == '1')
			{
				// Enable Tx/Rx Switching
				WSD_EnableBBTxRxSwitchingCtrl();
			}
			else if (pcOperand[0] == '0')
			{
				// Disable Tx/Rx Switching
				WSD_DisableBBTxRxSwitchingCtrl();
			}
			else
			{
				// Toggle Tx/Rx Switching
				if (trx.automaticTxRxSwitchingEnabled)
				{
					WSD_DisableBBTxRxSwitchingCtrl();
				}
				else
				{
					WSD_EnableBBTxRxSwitchingCtrl();
				}
			}
			break;
		}
	// Setup and enable the RF Loopback system
	// Example: E
	// ================================================================================
		case OP_LOOPBACK_ENABLE:
		{
			Lime_StartRFLoopbackMode();
			PrintStr("RF Loopback Mode Enabled.");
			__newline
			break;
		}
	// Toggle the TX/RX switch states
	// Example: I <t|r|T|R|c|C|A#>
	// ================================================================================
		case OP_SWAP_IQ_PHASE:
		{
			// check whether we're swapping the TX or RX polarity
			char * pcArg = pcOperand;
			if (*pcArg == 't')
			{
				Lime_TX_SwapIQPolarity();
			} else if (*pcArg == 'r')
			{
				Lime_RX_SwapIQPolarity();
			} else if (*pcArg == 'T')
			{
				Lime_TX_SwapIQInterleave();
			} else if (*pcArg == 'R')
			{
				Lime_RX_SwapIQInterleave();
			} else if (*pcArg == 'c')
			{
				Lime_FlipDACClkEdgePolarity();
			} else if (*pcArg == 'C')
			{
				Lime_FlipADCSampPhaseSel();
			} else if (*pcArg == 'A')
			{
				Lime_SetClkNonOverlapPhaseAdj(trx.clkNonOverlapAdj + 1 % 3);
			}

			break;
		}

	// Print help dialog to give command descriptions and bounds.
	// ================================================================================
		case OP_HELP:
		{
			Stel_PrintHelpDialog();
			break;
		}

	// Toggle between "FDD"-modd and TDD mode Tx/Rx switching
	// Example: z
	// ================================================================================
		case OP_TOGGLE_SW_METHOD:
		{
			// Disable switching interrupts
			WSD_DisableBBTxRxSwitchingCtrl();

			// Change switching mode: toggle on no arg, set to TDD mode if arg is '1'
			if ( (trx.TxRxSwitchingMode == TxRxSWMode_TDD) || (pcOperand[0] == '1'))
			{
				// Make safe TDD
				__LMS_SET_RX_TDD;

				// Enable FDD mode
				__LMS_PRIME_TRX_FDD;

				// Switch modes to FDD
				trx.TxRxSwitchingMode = TxRxSWMode_FDD;

				PrintStr("TxRxSWMode = FDD");
				__newline
			}
			else
			{
				// Make safe FDD
				__LMS_SET_RX_FDD;

				// Enable TDD mode
				__LMS_PRIME_TRX_TDD;

				// Switch modes to TDD
				trx.TxRxSwitchingMode = TxRxSWMode_TDD;

				PrintStr("TxRxSWMode = TDD");
				__newline
			}

			// Enable switching interrupts
			WSD_EnableBBTxRxSwitchingCtrl();

			break;
		}

	// Set Tx => Rx PA2 timeout counter length. This is under active
	// development, so we've written this into the firmware to let us
	// change later
	// Example: y 1000
	// ================================================================================
		case OP_SET_RXTXPA_TIMEOUT:
		{
			unsigned long ulVal;

			// Convert the value; the first bit selects the delay value
			ulVal = myDec2Long(pcOperand + 1);
			if (pcOperand[0] == '1')
			{
				// Set the transmit PLL settling time delay
				trx.ulTxRxPAOffTimeout = ulVal;
			}
			else if (pcOperand[0] == '0')
			{
				// Set the receive PLL settling time + RF PA off-time delay
				trx.ulRxTxPLLSettlingTimeout = ulVal;
			}
			else
			{
				// Um... try again.
				Stel_ThrowError("Bad Delay Select Bit");
				break;
			}
			PrintStr("Set PA2 switching timeout.");
			__newline
			break;
		}

	// Load calibration table row entry to SRAM or just display current table
	// Example: c 0 5 IQ_TX_S IQ_TX_C IQ_TX_M
	//			c B								- read table B
	//			c BIIQQ							- write table B, Rx LOFT Cal
	//			c BGGIIQQ						- write table B, Tx LOFT Cal @ Gain GG
	//          c BFFXXXXXXXXYYYYYYYYZZZZZZZZ0	- write table B, TxIQ Cal @ freq index FF
	//          c BFFXXXXXXXXYYYYYYYYZZZZZZZZ1	- write table B, RxIQ Cal @ freq index FF
	//			c								- read all tables
	// ================================================================================
		case OP_SET_RF_CALIB_TABLE:
		{
			uint8_t arglen = strlen(pcOperand);
//			unsigned long ulVal;
//			CalTable_t * pCalTable;

			if (Stel_ProcessCalibrationCommandString(pcOperand))
			{
				Stel_ThrowError("Bad Cal Arg");
			}
//			if (arglen == 0)
//			{
//				// Display the whole calibration table
//				Stel_PrintCalibrationTable(-1);
//			}
//			else if (arglen == 1)
//			{
//				// Display only calibration table ulVal
//				ulVal = myDec2Long(pcOperand);
//				Stel_PrintCalibrationTable((int)ulVal);
//			}
//			else if (arglen == 5)
//			{
//				// Load Rx LOFT
//				// c 01234
//				// c BXXYY
//				pCalTable = Stel_GetCalibrationTable();
//				ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);
//				if (ulVal == 0)
//				{
//					// Band 00 = UHF
//					pCalTable->UHFTable.rx_loft_cal.i =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 1, 2);
//					pCalTable->UHFTable.rx_loft_cal.q =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//				}
//				else if (ulVal == 1)
//				{
//					// Band 01 = WiFi
//					pCalTable->WiFiTable.rx_loft_cal.i =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 1, 2);
//					pCalTable->WiFiTable.rx_loft_cal.q =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//				}
//				else
//				{
//					Stel_ThrowError("Bad band #");
//				}
//			}
//			else if (arglen == 7)
//			{
//				// Load Tx LOFT
//				// c 0123456
//				// c BGGXXYY
//				pCalTable = Stel_GetCalibrationTable();
//				ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);
//				if (ulVal == 0)
//				{
//					// Band 00 = UHF
//					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//					pCalTable->UHFTable.tx_loft_cal[ulVal].i =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//					pCalTable->UHFTable.tx_loft_cal[ulVal].q =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
//				}
//				else if (ulVal == 1)
//				{
//					// Band 01 = WiFi
//					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//					pCalTable->WiFiTable.tx_loft_cal[ulVal].i =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//					pCalTable->WiFiTable.tx_loft_cal[ulVal].q =
//							(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
//				}
//				else
//				{
//					Stel_ThrowError("Bad band #");
//				}
//			}
//			else if (arglen == 28)
//			{
//				// Load IQ Imbalance
//				//   0000000000111111111122222222
//				// c 0123456789012345678901234567
//				// c B##XXXXXXXXYYYYYYYYZZZZZZZZ#
//				pCalTable = Stel_GetCalibrationTable();
//				ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);
//
//				// This is a Tx IQ-compensation value
//				if (pcOperand[27] == '0')
//				{
//					if (ulVal == 0)
//					{
//						// Band 00 = UHF
//						ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//						pCalTable->UHFTable.tx_iq_bb_cal[ulVal].smult =
//								Stel_hexSubstrToLong(pcOperand, 3, 10);
//						pCalTable->UHFTable.tx_iq_bb_cal[ulVal].cmult =
//								Stel_hexSubstrToLong(pcOperand, 11, 18);
//						pCalTable->UHFTable.tx_iq_bb_cal[ulVal].gmult =
//								Stel_hexSubstrToLong(pcOperand, 19, 26);
//					}
//					else if (ulVal == 1)
//					{
//						// Band 01 = WiFi
//						ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//						pCalTable->WiFiTable.tx_iq_bb_cal[ulVal].smult =
//								Stel_hexSubstrToLong(pcOperand, 3, 10);
//						pCalTable->WiFiTable.tx_iq_bb_cal[ulVal].cmult =
//								Stel_hexSubstrToLong(pcOperand, 11, 18);
//						pCalTable->WiFiTable.tx_iq_bb_cal[ulVal].gmult =
//								Stel_hexSubstrToLong(pcOperand, 19, 26);
//					}
//					else
//					{
//						Stel_ThrowError("Bad band #");
//					}
//				}
//				// This is an Rx IQ-compensation value
//				else if (pcOperand[27] == '1')
//				{
//					if (ulVal == 0)
//					{
//						// Band 00 = UHF
//						ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//						pCalTable->UHFTable.rx_iq_bb_cal[ulVal].smult =
//								Stel_hexSubstrToLong(pcOperand, 3, 10);
//						pCalTable->UHFTable.rx_iq_bb_cal[ulVal].cmult =
//								Stel_hexSubstrToLong(pcOperand, 11, 18);
//						pCalTable->UHFTable.rx_iq_bb_cal[ulVal].gmult =
//								Stel_hexSubstrToLong(pcOperand, 19, 26);
//					}
//					else if (ulVal == 1)
//					{
//						// Band 01 = WiFi
//						ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//						pCalTable->WiFiTable.rx_iq_bb_cal[ulVal].smult =
//								Stel_hexSubstrToLong(pcOperand, 3, 10);
//						pCalTable->WiFiTable.rx_iq_bb_cal[ulVal].cmult =
//								Stel_hexSubstrToLong(pcOperand, 11, 18);
//						pCalTable->WiFiTable.rx_iq_bb_cal[ulVal].gmult =
//								Stel_hexSubstrToLong(pcOperand, 19, 26);
//					}
//					else
//					{
//						Stel_ThrowError("Bad band #");
//					}
//				}
//				else
//				{
//					Stel_ThrowError("Append Tx/Rx selector to IQ vals");
//				}
//			}
//			else
//			{
//				char pcBuff[5];
//
//				Stel_ThrowError("unknown cal table operand");
//				myLong2Hex(arglen, pcBuff, 2);
//				PrintStr((unsigned char *)pcBuff);
//				__newline
//			}
			break;
		}

	//
	// Complete register dump of the LMS6002D configuration registers
	// Example: d
	// ================================================================================
		case OP_REGISTER_DUMP:
		{
			unsigned long ulReg;
			unsigned long ulVal;
			unsigned long ii;
			unsigned long jj;
			unsigned long kk;
			char pcBuff[15];

			PrintStr("======== LMS6002D WARP CORE DUMP ========");
			__newline
			//Memory block: [x000, x111]
			for (ii = 0; ii < 8; ii++)
			{
				//Reg Addr: [0000, 1111]
				for (jj = 0; jj < 16; jj++)
				{
					ulReg = (ii<<4) + jj;
					myLong2Hex(ulReg, pcBuff, 2);
					PrintStr((unsigned char *)pcBuff);

					PrintStr(" ");

					Lime_DisplayRegisterFieldVal(ulReg, 0xFF);

					PrintStr(" ");

					Lime_Read(ulReg, &ulVal);
					for (kk = 0; kk < 8; kk++)
					{
						// Mask out top bit & check
						if (ulVal & 0x80)
						{
							PrintStr("1");
						}
						else
						{
							PrintStr("0");
						}
						// Period separator
						if (kk == 3)
						{
							PrintStr(".");
						}
						// Left-shift for next comparison
						ulVal = ulVal << 1;
					}
					__newline
				}
			}
			PrintStr("================================");
			__newline

			break;
		}

	//
	// Commit the currently loaded calIbration table to flash memory.
	// Example: s
	// ================================================================================
		case OP_COMMIT_CALIB_TABLE:
		{
			if (Stel_AtomicCommitCalibrationTable())
			{
				Stel_ThrowError("FlashCommitFailed");
			} else
			{
				PrintStr("Calibration Table Saved");
				__newline
//				Stel_PrintCalibrationTable(-1);
			}
			break;
		}
	//
	// Single-entry Tx gain-setting function. This allocated optimal gain settings and
	// adjusts the TX LOFT calibration table appropriately.
	// Example: n <0, 56>
	// ================================================================================
		case OP_SET_TX_GAIN:
		{
			unsigned long ulVal = myDec2Long(pcOperand);
			if ( (ulVal > 56) )//|| (ulVal < 0) )
			{
				Stel_ThrowError("TX Gain OOB");
				break;
			}
			// Calibration showed that TxVGA gain was more consistent when G (TxVGA1)
			// was maximized first, before H (TxVGA2) was increased
			Lime_SetMasterTxGain(ulVal);
			break;
		}
	//
	// Increment or decrement the I or Q RX LOFT calibration value.
	// Example: <1, 2, 3, 4>
	// ================================================================================
		case ASCII_1:
		case ASCII_2:
		case ASCII_3:
		case ASCII_4:
		{
			unsigned long ulBuff;
			char cBuff[4];
			unsigned long target_register;
			int direction;

			if (ucOpCode == ASCII_1) {
				target_register = LMS_DCO_RXFE_I_R;
				direction = 0;
				PrintStr("Dec RXFE I DCO: ");
			} else if (ucOpCode == ASCII_2)
			{
				target_register = LMS_DCO_RXFE_I_R;
				direction = 1;
				PrintStr("Inc RXFE I DCO: ");
			} else if (ucOpCode == ASCII_3)
			{
				target_register = LMS_DCO_RXFE_Q_R;
				direction = 0;
				PrintStr("Dec RXFE Q DCO: ");
			} else //if (ucOpCode == ASCII_4)
			{
				target_register = LMS_DCO_RXFE_Q_R;
				direction = 1;
				PrintStr("Inc RXFE Q DCO: ");
			}

			// Increment/Decrement the target DCO DAC setting according to the key pressed.
			ulBuff = Lime_GetRXFE_DCO(target_register);

			if (direction) {
				// bound check upper bound
				if (ulBuff == 126) {
					Stel_ThrowError("OOB DCO DAC Value");

					// ignore this received char
					pBuff->numCharsInBuff--;
					PrintStr("wss$ ");
					break;
				} else {
					ulBuff++;
				}
			} else {
				// bound check lower bound
				if (ulBuff == 0) {
					Stel_ThrowError("OOB DCO DAC Value");

					// ignore this received char
					pBuff->numCharsInBuff--;
					PrintStr("wss$ ");
					break;
				} else {
					ulBuff--;
				}
			}

			// debug output
			myLong2Hex(ulBuff, cBuff, 2);
			Lime_SetRXFE_DCO(target_register, ulBuff);
			PrintStr((unsigned char *)cBuff);
			__newline
			break;
		}
	//
	// Set the WSD into DAC/ADC-only mode.
	// Example: U
	// ================================================================================
	case OP_SET_ADCDAC_MODE:
		{
			PrintStr("=========== !EVERYTHING IS AWESOME! ===========\r\n");
			PrintStr("DAC/ADC-Only Mode. Nothing else works.\r\n");
			PrintStr("You MUST power-cycle to completely undo this.\r\n");
			PrintStr("==> This is known not-working at this time <==\r\n");
			Lime_FuckingNukeEverything();
			break;
		}
	//
	// Increment or decrement the I or Q TX LOFT calibration value.
	// Example: <6, 7, 8, 9>
	// ================================================================================
		case ASCII_6:
		case ASCII_7:
		case ASCII_8:
		case ASCII_9:
		{
			unsigned long ulBuff;
			char cBuff[4];
			unsigned long target_register;
			int direction;

			if (ucOpCode == ASCII_6) {
				target_register = LMS_DCO_TXRF_I_R;
				direction = 0;
				PrintStr("Dec TXRF I DCO: ");
			} else if (ucOpCode == ASCII_7)
			{
				target_register = LMS_DCO_TXRF_I_R;
				direction = 1;
				PrintStr("Inc TXRF I DCO: ");
			} else if (ucOpCode == ASCII_8)
			{
				target_register = LMS_DCO_TXRF_Q_R;
				direction = 0;
				PrintStr("Dec TXRF Q DCO: ");
			} else //if (ucOpCode == ASCII_9)
			{
				target_register = LMS_DCO_TXRF_Q_R;
				direction = 1;
				PrintStr("Inc TXRF Q DCO: ");
			}

			// Increment/Decrement the target DCO DAC setting according to the key pressed.
			ulBuff = Lime_GetTXRF_DCO(target_register);
			if (direction) {
				if (!(ulBuff == 255))
						ulBuff++;
			} else {
				if (!(ulBuff == 0))
						ulBuff--;
			}

			// debug output
			myLong2Hex(ulBuff, cBuff, 2);
			Lime_SetTXRF_DCO(target_register, ulBuff);
			PrintStr((unsigned char *)cBuff);
			__newline
			break;
		}
	// Default Op Code handler - unrecognized opcode
	// ================================================================================
		default:
		{
			PrintStr("! Unrecognized OpCode !");
			__newline
			PrintStr("\r\n OP: ");
			PrintStr((unsigned char *)pBuff->cOpCodeBuff);
			PrintStr("\r\nCMD: ");
			PrintStr((unsigned char *)pBuff->cCmdBuff);
			__newline
			break;
		}
	}

	return;
}

//*****************************************************************************
//
// Sets flag to signal to main loop that one of the terminal devices has
// pending characters to process.
//
//*****************************************************************************
void
Stel_SetUSB0BufferHasCharsFlag()
{
	iUSB0BufferHasChars = 1;
}

//*****************************************************************************
//
// Sets flag to signal to main loop that the terminal buffers need to be reset
// and a terminal splash should be printed.
//
//*****************************************************************************
void
Stel_SetResetTerminalPendingFlag()
{
	g_cResetTerminalPendingFlag = 1;
}

////*****************************************************************************
////
//// Initialize the persistent FLASH calibration table driver & copy into SRAM.
////
//// \return A zero upon success. Non-zero on error.
////
////*****************************************************************************
//int
//Stel_InitFlashCalibrationTable(void)
//{
//	unsigned char * pucBuff;
//
////	char cBuff[10];
////	PrintStrUARTDebug("Tablesize HEX ");
////	myLong2Hex(sizeof(tCalib_DCO_LOFT_Table), cBuff, 1);
////	PrintStrUARTDebug((unsigned char *)cBuff);
////	PrintStrUARTDebug("\n\r");
////	PrintStrUARTDebug("Rowsize HEX ");
////	myLong2Hex(sizeof(tCalib_DCO_LOFT_Row), cBuff, 1);
////	PrintStrUARTDebug((unsigned char *)cBuff);
////	PrintStrUARTDebug("\n\r");
//
//	// This must be called first before anything dealing with flash parameters
//	FlashPBInit(FLASH_PB_START, FLASH_PB_END, sizeof(CalTable_t));
//
//	// Get the calibration table stored in flash; note that because this is
//	// a generic driver, it returns an unsigned char * rather than our type
//	pucBuff = FlashPBGet();
//
//	// Test for a non-initialized flash table or something wrong with it.
//	if (pucBuff == NULL)
//	{
////		PrintStrUARTDebug("! No Valid Param Blocks !\n\r");
////		__newline
//		return 1;
//	}
//
//	// Point the global table to the SRAM table. Now it's initialized.
//	return Stel_SetCalibrationTable((CalTable_t *)pucBuff);
//}

////*****************************************************************************
////
//// Returns a pointer to the SRAM calibration table.
////
//// \return A pointer to the mutable calibration table. NULL on error or bad table.
////
////*****************************************************************************
//CalTable_t *
//Stel_GetCalibrationTable(void)
//{
//	// Test for a non-initialized flash table or something wrong with it.
////	if (psCalibrationTable == NULL)
////	{
//////		PrintStr("Returning NULL Table");
////		PrintStrUARTDebug("! NULL Table 1\n\r");
////		__newline
////		return NULL;
////	}
//
//	// Our work here is done.
//	return &g_CalTable;
//}

////*****************************************************************************
////
//// Set the SRAM global calibration table to the passed table (via ptrs)
////
//// \return A zero upon success. Non-zero on error.
////
////*****************************************************************************
//int
//Stel_SetCalibrationTable(CalTable_t * psNewTable)
//{
//	// Test for a non-initialized flash table or something wrong with it.
//	if (psNewTable == NULL)
//	{
//		Stel_ThrowError("NULL Table 2");
//		PrintStrUARTDebug("! NULL Table 2\n\r");
//		return 1;
//	}
//
//	// Otherwise, set the global table to the new table
//	memcpy(&g_CalTable, psNewTable, sizeof(CalTable_t));
//
//	return 0;
//}

////*****************************************************************************
////
//// Overwrite the calibration table (if any) stored in FLASH with the current
//// one in SRAM. Error checking included.
////
//// Because of the fault-tolerant nature of the flashpb driver library, this is
//// an atomic operation. Although it's not perfect because only one FLASH page
//// is currently allocated. For perfect fault-tolerance, we'd have to allocate
//// TWO pages.
////
//// \return A zero upon success. Non-zero on error.
////
////*****************************************************************************
//int
//Stel_AtomicCommitCalibrationTable()
//{
//	// Test for a non-initialized flash table or something wrong with it.
////	if (psCalibrationTable == NULL)
////	{
////		PrintStr("! Can't Commit NULL Table !");
////		PrintStrUARTDebug("! Can't Commit Table\n\r");
////		__newline
////		return 1;
////	}
//
//	// Write the current global calibration table to FLASH.
//	FlashPBSave((unsigned char *)&g_CalTable);
//
//	// Reload the SRAM table in order to update checksum and seqno.
//	Stel_InitFlashCalibrationTable();
//
//	return 0;
//}

////*****************************************************************************
////
//// Print the current RF Calibration Table contents to the terminal.
////
//// \param band_no integer band index to print starting at index 0. Index -1 means print
////                all tables.
////
////*****************************************************************************
//void
//Stel_PrintCalibrationTable(int band_no)
//{
//	char pcBuff[10];
//#define NUMCOLS 9			//programmatic hack.
//#define MAXVAL 25			// hack 2
//
//	CalTable_t * pCalTable;
//	uint32_t uiBand;
//	uint32_t uiFrequency;
//	uint32_t uiIndex;
//
//	pCalTable = Stel_GetCalibrationTable();
//	if (pCalTable == NULL)
//	{
//		Stel_ThrowError("Cal Table Uninitialized");
//		return;
//	}
//
//	// Print table header w/ table number
//	PrintStr("============ Table ");
//	myLong2Hex((unsigned long)pCalTable->ucSeqNo, pcBuff, 3);
//	PrintStr((unsigned char *)pcBuff);
//	PrintStr(" ===== Size ");
//	myLong2Hex((unsigned long)sizeof(CalTable_t), pcBuff, 4);
//	PrintStr((unsigned char *)pcBuff);
//	PrintStr(" ============");
//	__newline
//
//	if ((band_no == -1)||(band_no == 0))
//	{
//		uiBand = 0;
//
//		// Print band header w/ band number
//		PrintStr(" ==================== Band 00 ====================");
//		__newline
//		// Print the RX LOFT Values
//		PrintStr("RX_I RX_Q");
//		__newline
//		PrintStr("0x");
//		myLong2Hex(pCalTable->UHFTable.rx_loft_cal.i, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(" 0x");
//		myLong2Hex(pCalTable->UHFTable.rx_loft_cal.q, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		__newline
//		// Print the TX LOFT Values in a 2-column table
//		PrintStr("GAIN TX_I TX_Q   GAIN TX_I TX_Q   GAIN TX_I TX_Q");
//		__newline
//		for (uiIndex = 0; uiIndex < NUMCOLS; uiIndex++)
//		{
//			// Print Tx LOFT setting for [0-Num/3]
//			PrintStr(" ");
//			myLong2Hex(uiIndex, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr("  0x");
//			myLong2Hex(pCalTable->UHFTable.tx_loft_cal[uiIndex].i, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr(" 0x");
//			myLong2Hex(pCalTable->UHFTable.tx_loft_cal[uiIndex].q, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//
//			// Print Tx LOFT setting for [Num/3, Num*2/3]
//			PrintStr("    ");
//			myLong2Hex(uiIndex + NUMCOLS, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr("  0x");
//			myLong2Hex(pCalTable->UHFTable.tx_loft_cal[uiIndex + NUMCOLS].i,
//					pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr(" 0x");
//			myLong2Hex(pCalTable->UHFTable.tx_loft_cal[uiIndex + NUMCOLS].q,
//					pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//
//			// Print Tx LOFT setting for [Num*2/3, Num]
//			// Don't print OOB values if the columns aren't equal.
//			if (uiIndex + 2*NUMCOLS <= MAXVAL) //RYAN
//			{
//				PrintStr("    ");
//				myLong2Hex(uiIndex + 2*NUMCOLS, pcBuff, 2);
//				PrintStr((unsigned char *)pcBuff);
//				PrintStr("  0x");
//				myLong2Hex(pCalTable->UHFTable.tx_loft_cal[uiIndex + 2*NUMCOLS].i,
//						pcBuff, 2);
//				PrintStr((unsigned char *)pcBuff);
//				PrintStr(" 0x");
//				myLong2Hex(pCalTable->UHFTable.tx_loft_cal[uiIndex + 2*NUMCOLS].q,
//						pcBuff, 2);
//				PrintStr((unsigned char *)pcBuff);
//			}
//			__newline
//		}
//
//		// Print the TX IQ Imbalance values
//		PrintStr("FREQUENCY   TX_SINMULT  TX_COSMULT  TX_GAINMULT");
////		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000");
//		__newline
//		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
//		{
//			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
//			Stel_PrintReg(uiFrequency, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->UHFTable.tx_iq_bb_cal[uiIndex].smult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->UHFTable.tx_iq_bb_cal[uiIndex].cmult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->UHFTable.tx_iq_bb_cal[uiIndex].gmult, pcBuff);
//			__newline
//		}
//
//		// Print the RX IQ Imbalance values
//		PrintStr("FREQUENCY   RX_SINMULT  RX_COSMULT  RX_GAINMULT");
////		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000");
//		__newline
//		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
//		{
//			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
//			Stel_PrintReg(uiFrequency, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->UHFTable.rx_iq_bb_cal[uiIndex].smult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->UHFTable.rx_iq_bb_cal[uiIndex].cmult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->UHFTable.rx_iq_bb_cal[uiIndex].gmult, pcBuff);
//			__newline
//		}
//	}
//
//	if ((band_no == -1)||(band_no == 1))
//	{
//		uiBand = 1;
//
//		// Print band header w/ band number
//		PrintStr(" ==================== Band 01 ====================");
//		__newline
//		// Print the RX LOFT Values
//		PrintStr("RX_I RX_Q");
//		__newline
//		PrintStr("0x");
//		myLong2Hex(pCalTable->WiFiTable.rx_loft_cal.i, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(" 0x");
//		myLong2Hex(pCalTable->WiFiTable.rx_loft_cal.q, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		__newline
//		// Print the TX LOFT Values in a 2-column table
//		PrintStr("GAIN TX_I TX_Q   GAIN TX_I TX_Q   GAIN TX_I TX_Q");
//		__newline
//		for (uiIndex = 0; uiIndex < NUMCOLS; uiIndex++)
//		{
//			// Print Tx LOFT setting for [0-Num/3]
//			PrintStr(" ");
//			myLong2Hex(uiIndex, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr("  0x");
//			myLong2Hex(pCalTable->WiFiTable.tx_loft_cal[uiIndex].i, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr(" 0x");
//			myLong2Hex(pCalTable->WiFiTable.tx_loft_cal[uiIndex].q, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//
//			// Print Tx LOFT setting for [Num/3, Num*2/3]
//			PrintStr("    ");
//			myLong2Hex(uiIndex + NUMCOLS, pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr("  0x");
//			myLong2Hex(pCalTable->WiFiTable.tx_loft_cal[uiIndex + NUMCOLS].i,
//					pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//			PrintStr(" 0x");
//			myLong2Hex(pCalTable->WiFiTable.tx_loft_cal[uiIndex + NUMCOLS].q,
//					pcBuff, 2);
//			PrintStr((unsigned char *)pcBuff);
//
//			// Print Tx LOFT setting for [Num*2/3, Num]
//			// Don't print beyond MAXVAL if columns aren't even
//			if (uiIndex + 2*NUMCOLS <= MAXVAL) //RYAN
//			{
//				PrintStr("    ");
//				myLong2Hex(uiIndex + 2*NUMCOLS, pcBuff, 2);
//				PrintStr((unsigned char *)pcBuff);
//				PrintStr("  0x");
//				myLong2Hex(pCalTable->WiFiTable.tx_loft_cal[uiIndex + 2*NUMCOLS].i,
//						pcBuff, 2);
//				PrintStr((unsigned char *)pcBuff);
//				PrintStr(" 0x");
//				myLong2Hex(pCalTable->WiFiTable.tx_loft_cal[uiIndex + 2*NUMCOLS].q,
//						pcBuff, 2);
//				PrintStr((unsigned char *)pcBuff);
//			}
//			__newline
//		}
//
//		// Print the TX IQ Imbalance values
//		PrintStr("FREQUENCY   TX_SINMULT  TX_COSMULT  TX_GAINMULT");
////		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000");
//		__newline
//		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
//		{
//			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
//			Stel_PrintReg(uiFrequency, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->WiFiTable.tx_iq_bb_cal[uiIndex].smult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->WiFiTable.tx_iq_bb_cal[uiIndex].cmult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->WiFiTable.tx_iq_bb_cal[uiIndex].gmult, pcBuff);
//			__newline
//		}
//
//		// Print the RX IQ Imbalance values
//		PrintStr("FREQUENCY   RX_SINMULT  RX_COSMULT  RX_GAINMULT");
////		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000");
//		__newline
//		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
//		{
//			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
//			Stel_PrintReg(uiFrequency, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->WiFiTable.rx_iq_bb_cal[uiIndex].smult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->WiFiTable.rx_iq_bb_cal[uiIndex].cmult, pcBuff);
//			PrintStr(" ");
//			Stel_PrintReg(pCalTable->WiFiTable.rx_iq_bb_cal[uiIndex].gmult, pcBuff);
//			__newline
//		}
//	}
//
//	// Done printing!
//	return;
//}

///*
// * Function to convert a specified substring to a long and return that value.
// *
// * \param ucStr the original string to parse
// * \param startInd the starting index of the substring
// * \param stopInd the ending index of the substring; the converted substring
// *                will INCLUDE this index
// *
// * \return integer representation of the parsed substring
// */
//uint32_t
//Stel_hexSubstrToLong(char * pcStr, uint32_t startInd, uint32_t stopInd)
//{
//	char pcBuff[20];
//	int ii;
//
//	for (ii = 0; ii <= stopInd - startInd; ii++)
//	{
//		pcBuff[ii] = pcStr[startInd + ii];
//	}
//	// Don't need to null terminate b/c hex string have explicit length.
////	pucBuff[stopInd - startInd + 1] = '\0';
//	return myHex2Long(pcBuff, stopInd - startInd + 1);
//}

////*****************************************************************************
////
//// Print the passed 32-bit register value to the current active terminal.
//// Format is: 0x1234.5678
////
//// \param regval the value to print
//// \param a pointer to a character buffer that is at least 5-chars wide
////
////*****************************************************************************
//void
//Stel_PrintReg(uint32_t regval, char *pcBuff)
//{
//	PrintStr("0x");
//	myLong2Hex( (regval & 0xFFFF0000) >> 16, pcBuff, 4);
//	PrintStr((unsigned char *)pcBuff);
//	PrintStr(".");
//	myLong2Hex( (regval & 0x0000FFFF), pcBuff, 4);
//	PrintStr((unsigned char *)pcBuff);
//}

//*****************************************************************************
//
// Print a help dialog describing the format and purpose of each of the Stellaris
// terminal opcodes.
//
//*****************************************************************************
void
Stel_PrintHelpDialog(void)
{
	PrintStr("===============================================================\r\n");
	PrintStr("============  Welcome to the (W)URC (S)PI (S)hell  ============");
	__newline
	PrintStr("All WURC command opcodes are one char + operand. Submit with ENTER key.");
	__newline
	PrintStr("Do not type any spaces. Numeric opcodes are special. A list of opcodes:");
	__newline
	PrintStr("> h : print this help menu");
	__newline
	__newline
	PrintStr("> c <#>: display calibration table #, leave blank for all");
	__newline
	PrintStr("  c <#GGXXYYD> : edit volatile LOFT calibration");
	__newline
	PrintStr("              # = Band (0 = UHF, 1 = 2.4 GHz WiFi");
	__newline
	PrintStr("             GG = Tx: Gain, Rx: Freq Index");
	__newline
	PrintStr("             XX = I LOFT Compensation, hex");
	__newline
	PrintStr("             YY = Q LOFT Compensation, hex");
	__newline
	PrintStr("              D = [0,1] = [Tx,Rx]");
	__newline
	PrintStr("  c <#FFAAAAAAAABBBBBBBBCCCCCCCCD> : edit volatile IQ calibration");
	__newline
	PrintStr("              # = Band (0 = UHF, 1 = 2.4 GHz WiFi");
	__newline
	PrintStr("             FF = Freq Index");
	__newline
	PrintStr("       AAAAAAAA = Tx BB Correction Sine, hex");
	__newline
	PrintStr("       BBBBBBBB = Tx BB Correction Cosine, hex");
	__newline
	PrintStr("       CCCCCCCC = Tx BB Correction Gain, hex");
	__newline
	PrintStr("              D = [0,1] = [Tx,Rx]");
	__newline
	PrintStr("> s : commit volatile calibration tables to non-volatile memory");
	__newline
	__newline
	PrintStr("> A : run WURC initialization/calibration sequence (runs on power-on)");
	__newline
	PrintStr("> F : run core LMS6002D calibration sequence (dev only)");
	__newline
	PrintStr("> C <T##BB> : set channel macro");
	__newline
	PrintStr("              T = 'A' (US UHF), 'E' (EU UHF), 'W' (802.11g)");
	__newline
	PrintStr("             ## = Channel Number");
	__newline
	PrintStr("             BB = Bandwidth [05, 10, 20]");
	__newline
	PrintStr("> x <0/1> : Toggle or set BB Tx/Rx switching control (default = off)");
	__newline
	PrintStr("> i : get trx info, test LMS SPI, and WSD serial (DEPRECATED)");
	__newline
	PrintStr("> j : get trx info, printed as a JSON string");
	__newline
	__newline
	PrintStr("> r <HH> : read LMS6002D configuration register 0xHH");
	__newline
	PrintStr("> w <HHKK> : write 0xKK to LMS6002D configuration register 0xHH");
	__newline
	PrintStr("> l : toggle red micro-controller error indicator LED");
	__newline
	PrintStr("> e : clear WSD error indicator LED");
	__newline
	PrintStr("> u : enable/disable automatic calibration loading (dev only)");
	__newline
//	PrintStr("> t : toggle RF PAs (deprecated: use N and M instead)");
//	__newline
	PrintStr("> B <kHz> : set RX center frequency");
	__newline
	PrintStr("> D <kHz> : set TX center frequency");
	__newline
	PrintStr("> R : toggle LMS soft RX enable (default = enable)");
	__newline
	PrintStr("> S : toggle LMS soft TX enable (default = enable)");
	__newline
	PrintStr("> X : soft LMS6002D reset, all WURC PAs deactivated");
	__newline
	PrintStr("> U : set LMS6002D to ADC/DAC-only mode (EXPERIMENTAL)");
	__newline
	__newline
	PrintStr("> T <0/1> : toggle or set LMS6002D RXOUTSW state (default = open)");
	__newline
	PrintStr("> P : print current loaded Rx/Tx LOFT DCO values");
	__newline
	PrintStr("> E : enable RF loop-back mode (DEPRECATED)");
	__newline
	PrintStr("> I <t|r> : swap the I/Q polarity of Tx or Rx");
	__newline
	PrintStr("    <T|R> : swap Tx/Rx I/Q Interleave mode");
	__newline
	PrintStr("        c : swap DAC Clk Edge Pol");
	__newline
	PrintStr("        C : swap ADC samp phase sel");
	__newline
	PrintStr("        A : advance clk non-overlap adj");
	__newline
	__newline
	PrintStr("> d : hex dump of all LMS6002D configuration registers");
	__newline
	PrintStr("> [0, 1, 2, 3] : RX [Dec I, Inc I, Dec Q, Inc Q] - Rx LOFT DCO DACs");
	__newline
	PrintStr("> [6, 7, 8, 9] : TX [Dec I, Inc I, Dec Q, Inc Q] - Tx LOFT DCO DACs");
	__newline
	PrintStr("> N : Set WURC as static Transmitter (setting \"x1\" overrides this)");
	__newline
	PrintStr("> M : Set WURC as static Receiver (setting \"x1\" overrides this)");
	__newline
	__newline
	PrintStr("> Z <#>: set transceiver mode");
	__newline
	PrintStr("         0 = Normal RF Input");
	__newline
	PrintStr("         1 = AUX RF Input (dev only)");
	__newline
	PrintStr("         2 = Enable RF Loopback");
	__newline
	PrintStr("         3 = Enable BB Loopback");
	__newline
	__newline
	PrintStr("> V <0:15> : set Transmit BB LPF bandwidth");
	__newline
	PrintStr("> W <0:15> : set Receive BB LPF bandwidth");
	__newline
	__newline
	PrintStr("> y [0/1]<#> : (1) set Tx PLL delay count to #");
	__newline
	PrintStr(">		         (0) set Rx PLL/Tx PA transition delay count to #");
	__newline
	PrintStr("> z [0/1] : (1) Tx/Rx switching mode = FDD (default)");
	__newline
	PrintStr(">           (0) Tx/Rx switching mode = TDD (dev only)");
	__newline
	__newline
	PrintStr("======== Tx Gains: use 'n' for production =======");
	__newline
	PrintStr("> n <0:56> : set overall transmit gain (LOGLIN)");
	__newline
	PrintStr("             Note: maximum linear gain is about 42 dB");
	__newline
	PrintStr("> G <0:31> : set Transmit VGA1 gain only (LOGLIN) DEPRECATED");
	__newline
	PrintStr("> H <0:25> : set Transmit VGA2 gain only (LOGLIN) DEPRECATED");
	__newline
	__newline
	PrintStr("====== Rx Gains: enable AGC for production ======");
	__newline
	PrintStr("> J <0:120> : set Receive VGA1 gain (LIN) DEPRECATED");
	__newline
	PrintStr("> K <0:10> : set Receive VGA2 gain (LOGLIN) DEPRECATED");
	__newline
	PrintStr("> L <0:2> : set Receive LNA gain (LOGLIN) DEPRECATED");
	__newline
	PrintStr("===============================================================\r\n");
	PrintStr("===============================================================\r\n");
}

//*****************************************************************************
//
// Print a JSON-formatted string containing all the data for everything.
//
//*****************************************************************************
void
Stel_PrintStatusJSON(TerminalBuff_t* pBuff)
{
	UHFCalTable_t * pUHFCalTable = Stel_GetUHFCalTablePointer();
	WiFiCalTable_t * pWiFiCalTable = Stel_GetWiFiCalTablePointer();

	g_activeTermBuff = pBuff;

	// chip information is stored in CPUID and DID0
	// respectively, this is ARM version information
	// and Stellaris chip version information
	unsigned long ulCPUID = NVIC_CPUID_R;
	unsigned long ulDID0 = SYSCTL_DID0_R;
	unsigned long ulARMVariant = (ulCPUID & NVIC_CPUID_VAR_M) >> 20;
	unsigned long ulARMRevision = (ulCPUID & NVIC_CPUID_REV_M);
	unsigned long ulARMPartNo = (ulCPUID & NVIC_CPUID_PARTNO_M) >> 4;
	unsigned long ulStellarisClass = (ulDID0 & SYSCTL_DID0_CLASS_M) >> 16;
	unsigned long ulStellarisMajorRev = (ulDID0 & SYSCTL_DID0_MAJ_M) >> 8;
	unsigned long ulStellarisMinorRev = (ulDID0 & SYSCTL_DID0_MIN_M);
	unsigned long ulSerialNo = WSD_GetSerialNumber();
	unsigned long ulBuff;
	char pcBuff[9];
	unsigned char pucBuff[13];

	PrintStr("  {");
	__newline
		// HARDWARE PARAMS =============================================
		PrintStr("   \"hw_params\": {");
		__newline
			PrintStr("    \"stellaris_class\": \"");
			myLong2Hex(ulStellarisClass, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("\",");
			__newline

			PrintStr("    \"version\": \"");
			myLong2Hex(ulStellarisMajorRev, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(".");
			myLong2Hex(ulStellarisMinorRev, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("\",");
			__newline

			PrintStr("    \"arm_core_part\": \"");
			myLong2Hex(ulARMPartNo, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("\",");
			__newline

			PrintStr("    \"arm_core_version\": \"");
			myLong2Hex(ulARMVariant, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(".");
			myLong2Hex(ulARMRevision, pcBuff, 1);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("\",");
			__newline

			PrintStr("    \"xceiver_spi_status\": \"");
			if (Lime_ConnectionLost()) {
				PrintStr("ERROR CHECK LMS SPI ERROR");
				PrintStr("\",");
				__newline
			} else {
				PrintStr("ok");
				PrintStr("\",");
				__newline

				PrintStr("    \"lms6002d_silicon_ver\": \"");
				Lime_Read(LMS_CHIP_INFO_R, &ulBuff);
				ltoa((ulBuff & LMS_VERSION_MASK) >> 4, pcBuff);
				PrintStr((unsigned char*)pcBuff);
				PrintStr(".");
				ltoa(ulBuff & LMS_REVISION_MASK, pcBuff);
				PrintStr((unsigned char*)pcBuff);
				PrintStr("\",");
				__newline
			}

			PrintStr("    \"cal_table_status\": \"");
//					pCalTable = Stel_GetCalibrationTable();
			if (pUHFCalTable == NULL || pWiFiCalTable == NULL) {
				PrintStr("UNINITIALIZED");
			} else if (pUHFCalTable->tx_iq_bb_cal[0].cmult == 0) {
				PrintStr("UHF ALL ZEROS");
			} else if (pUHFCalTable->rx_iq_bb_cal[0].cmult == 0) {
				PrintStr("UHF RX ALL ZEROS");
			} else if (pWiFiCalTable->tx_iq_bb_cal[0].cmult == 0) {
				PrintStr("WiFi ALL ZEROS");
			} else if (pWiFiCalTable->rx_iq_bb_cal[0].cmult == 0) {
				PrintStr("WiFi RX ALL ZEROS");
			} else {
				PrintStr("ok");
			}
			PrintStr("\",");
			__newline

#if defined(__VOLO_WURC_REV_RED__)
			PrintStr("    \"target_hw_rev\": \"4_red\",");
#elif defined(__VOLO_WURC_REV_GREEN__)
			PrintStr("    \"target_hw_rev\": \"3_green\",");
#elif defined(__VOLO_WURC_REV_BLUE__)
			PrintStr("    \"target_hw_rev\": \"2_blue\",");
#else
			PrintStr("    \"target_hw_rev\": \"UNKNOWN\",");
#endif
			__newline
			__newline

			// NOTE: the Python wrapper looks for "Serial" and a hex string
			// of arbitrary length on the same line. Do not distrupt that.
			PrintStr("    \"wsd_hex_Serial\": \"#");
			myLong2Hex(ulSerialNo, pcBuff, 5);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("\"");
			__newline
		PrintStr("   },");
		__newline
		// END HARDWARE PARAMS
		// SOFTWARE PARAMS =============================================
		PrintStr("   \"sw_params\": {");
		__newline
			PrintStr("    \"firmware_version\": \"");
			PrintStr(pucWSDFirmVersion);
			PrintStr("\",");
			__newline

			PrintStr("    \"compile_date\":     \"");
			PrintStr(pucWSDFirmRevDate);
			PrintStr(" ");
			PrintStr(pucWSDFirmRevTime);
			PrintStr("\"");
			__newline
		PrintStr("   },");
		__newline
		// END SOFTWARE PARAMS
		// RADIO PARAMS ================================================
		PrintStr("   \"xcvr_params\": {");
		__newline

			PrintStr("    \"uhf_pa1\": \"");
			unsigned long ulTest = GPIOPinRead(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN);
			PrintStr(ulTest ? (unsigned char*)"ON\"," : (unsigned char*)"off\",");
			__newline

			PrintStr("    \"uhf_pa2\": \"");
			ulTest = GPIOPinRead(CTRL_UHF_PA2_BASE, CTRL_UHF_PA2_PIN);
			PrintStr(ulTest ? (unsigned char*)"ON\"," : (unsigned char*)"off\",");
			__newline

			PrintStr("    \"uhf_lna\": \"");
			ulTest = GPIOPinRead(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN);
			PrintStr(ulTest ? (unsigned char*)"ON\"," : (unsigned char*)"off\",");
			__newline

			PrintStr("    \"wifi_pa\": \"");
			ulTest = GPIOPinRead(CTRL_WIFI_PA_BASE, CTRL_WIFI_PA_PIN);
			PrintStr(ulTest ? (unsigned char*)"ON\"," : (unsigned char*)"off\",");
			__newline

			PrintStr("    \"uhf_ant_sw\":  \"tx_to_");
			ulTest = GPIOPinRead(CTRL_ANT_BASE, CTRL_UHF_ANT_PIN);
			PrintStr(ulTest ? (unsigned char*)"ant2\"," : (unsigned char*)"ANT1\",");
			__newline

			PrintStr("    \"wifi_ant_sw\": \"tx_to_");
			ulTest = GPIOPinRead(CTRL_ANT_BASE, CTRL_WIFI_ANT_PIN);
			PrintStr(ulTest ? (unsigned char*)"ant2\"," : (unsigned char*)"ANT1\",");
			__newline
			__newline

			PrintStr("    \"tx_vga_1\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_GAIN_TXVGA1_R, LMS_GAIN_TXVGA1_M);
			PrintStr("\",");
			__newline

			PrintStr("    \"tx_vga_2\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_GAIN_TXVGA2_R, LMS_GAIN_TXVGA2_M);
			PrintStr("\",");
			__newline
			__newline

			PrintStr("    \"lna\":      \"0x");
			Lime_DisplayRegisterFieldVal(LMS_LNA_CTRL_R, LMS_GAIN_LNA_M);
			PrintStr("\",");
			__newline

			PrintStr("    \"rx_vga_1\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_GAIN_RXVGA1_R, LMS_GAIN_RXVGA1_M);
			PrintStr("\",");
			__newline

			PrintStr("    \"rx_vga_2\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_GAIN_RXVGA2_R, LMS_GAIN_RXVGA2_M);
			PrintStr("\",");
			__newline
			__newline

			PrintStr("    \"tx_lpf\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_TXLPF_BASE + LMS_LPF_BWC_OFFSET, LMS_LPF_BWC_M);
			PrintStr("\",");
			__newline

			PrintStr("    \"rx_lpf\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_RXLPF_BASE + LMS_LPF_BWC_OFFSET, LMS_LPF_BWC_M);
			PrintStr("\",");
			__newline
			__newline

			PrintStr("    \"lna_sel\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_LNA_CTRL_R, LMS_LNA_CTRL_LNASEL_M);
			PrintStr("\",");
			__newline

			PrintStr("    \"vga2_sel\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_TXVGA2_CTRL_R, LMS_TXVGA2_USR_CTRL_M);
			PrintStr("\",");
			__newline

			PrintStr("    \"rf_loopb_sel\": \"0x");
			Lime_DisplayRegisterFieldVal(LMS_LOOPBACK_CTRL_R, LMS_LOOPBACK_LBRFEN_M);
			PrintStr("\",");
			__newline

			PrintStr("    \"tx_freq_hex\": \"");
			Stel_PrintReg(trx.ulCurrentTxFreq, (char *)pucBuff);
			PrintStr("\",");
			__newline

			PrintStr("    \"rx_freq_hex\": \"");
			Stel_PrintReg(trx.ulCurrentRxFreq, (char *)pucBuff);
			PrintStr("\",");
			__newline

			PrintStr("    \"tx_gain_db\": \"0x");
			myLong2Hex(trx.ulCurrentTxGain, (char *)pucBuff, 2);
			PrintStr(pucBuff);
			PrintStr("\",");
			__newline
			__newline

			PrintStr("    \"auto_cal_loading\":    \"");
			PrintStr(trx.automaticCalLoadingEnabled == TRUE ? (unsigned char*)"loading_enabled\"," : (unsigned char*)"off\",");
			__newline

			PrintStr("    \"auto_txrx_switching\": \"");
			PrintStr(trx.automaticTxRxSwitchingEnabled ? (unsigned char*)"txrx_sw_enabled\"," : (unsigned char*)"off\",");
			__newline

			PrintStr("    \"txrx_switching_mode\": \"");
			PrintStr(trx.TxRxSwitchingMode ? (unsigned char*)"TDD\"," : (unsigned char*)"FDD\",");
			__newline

			PrintStr("    \"calibration_mode\":    \"");
			PrintStr(trx.CalibrationMode ? (unsigned char*)"Loopback\"," : (unsigned char*)"Normal\",");
			__newline

			PrintStr("    \"rx_input_mode\":       \"");
			PrintStr(trx.RxInputMode ? (unsigned char*)"Normal\"," : (unsigned char*)"AUX\",");
			__newline

			PrintStr("    \"transmit_band\":       \"");
			PrintStr(trx.isTransmittingInUHFBand ? (unsigned char*)"UHF\"," : (unsigned char*)"ISM\",");
			__newline

			PrintStr("    \"receive_band\":        \"");
			PrintStr(trx.isReceivingInUHFBand ? (unsigned char*)"UHF\"," : (unsigned char*)"ISM\",");
			__newline

			PrintStr("    \"direct_lna_ctrl\":     \"");
			PrintStr(trx.directCTRL_LNAIsActive ? (unsigned char*)"ACTIVE\"," : (unsigned char*)"deactivated\",");
			__newline
			__newline

			// Get the contents of the MISC_CTRL register, which contains the
			// settings for IQ polarity selection and IQ frame-alignment on the
			// Lime digital interface.
			Lime_Read(LMS_MISC_CTRL_R, &ulBuff);

			PrintStr("    \"bb_iface_config\": \"0x");
			myLong2Hex(ulBuff, (char *)pucBuff, 2);
			PrintStr(pucBuff);
			PrintStr("\",");
			__newline
//
//			PrintStr("   \"tx_fsync\":     ");
//			PrintStr( (ulBuff & LMS_MISC_TX_FSYNC_POL_M) ? (unsigned char*)"1," : (unsigned char*)"0,");
//			__newline
//			PrintStr("   \"rx_fsync\":     ");
//			PrintStr( (ulBuff & LMS_MISC_RX_FSYNC_POL_M) ? (unsigned char*)"1," : (unsigned char*)"0,");
//			__newline
//			PrintStr("   \"tx_iq_interl\": ");
//			PrintStr( (ulBuff & LMS_MISC_TX_INTERLEAVE_M) ? (unsigned char*)"1," : (unsigned char*)"0,");
//			__newline
//			PrintStr("   \"rx_iq_interl\": ");
//			PrintStr( (ulBuff & LMS_MISC_RX_INTERLEAVE_M) ? (unsigned char*)"1," : (unsigned char*)"0,");
//			__newline
//			PrintStr("   \"adc_phase_sel\": ");
//			PrintStr( (ulBuff & LMS_MISC_ADC_SAMPLE_PHASE_SEL_M) ? (unsigned char*)"1," : (unsigned char*)"0,");
//			__newline
//			PrintStr("   \"dac_clk_edge_pol\": ");
//			PrintStr( (ulBuff & LMS_MISC_DAC_CLK_EDGE_POL_M) ? (unsigned char*)"1," : (unsigned char*)"0,");
//			__newline
//			PrintStr("   \"clk_non_ovl_adj_ps\": ");
//			switch ((ulBuff & LMS_MISC_CLK_NON_OVERLAP_ADJ_M))
//			{
//				case 3:
//					PrintStr("300,"); break;
//				case 2:
//					PrintStr("150,"); break;
//				case 1:
//					PrintStr("450,"); break;
//				case 0:
//					PrintStr("0,"); break;
//			}
//			__newline
			PrintStr("    \"trx_mode\": ");
			switch (trx.TRXState)
			{
				case RX_NORMAL:
					PrintStr("\"normal\""); break;
				case RX_BYPASS_INT_LNA:
					PrintStr("\"bypass lna\""); break;
				case RX_LOOPBACK_RF:
					PrintStr("\"rf loopb\""); break;
				case RX_LOOPBACK_BB:
					PrintStr("\"bb loopb\""); break;
			}

		__newline
		PrintStr("   }");
		// END RADIO PARAMS
		__newline
	PrintStr("  }");
}


//*****************************************************************************
//
// Main function. Code execution starts here.
//
//*****************************************************************************
int

 main(void) {
	// System clock should be 80 MHz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	// LED Indicator (PA7), SSI0 (PA[2-5]), UART0 (PA[0-1]), PKT_DET (PA6)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	// PA/SW CTRL Lines (PC[4-7], PE4) and JTAG (PC[0-3])
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	// Enable Port B for FPGA communications
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// Enable processor interrupts
	ROM_IntMasterEnable();

	// always enable the UART0 peripheral for debug
	SetupUART0();
	ROM_IntEnable(INT_UART0);

	// Enable the USB0 peripheral utilizing the USB CDC Driver supplied
	// by TI.
	SetupUSB0();

	// This directly masters the Lime Micro LMS6002D transceiver through
	// the SSI0 serial peripheral on pins PC[0-3]. Note that the 4-pin SSI
	// control lines are MUXed between the Stellaris and the FPGA. Default
	// control goes to the Stellaris.
	SetupSSI0();

	// Enables interrupts of PB0 (RxEn) and PB1 (TxEN) for FPGA control
 	SetupFPGA_GPIO();
// 	ROM_IntEnable(INT_GPIOB); // this is done as last thing after radio boots

 	// Ititialize the two terminal buffers, supplying both with their
 	// appropriate callback functions.
 	volo_initTermBuff(&sUSB0Buff, &PrintStrUSB);
 	volo_initTermBuff(&sUART0Buff, &PrintStrUART);

 	// Clear any previous chars in the buffer and print the terminal prompt.
 	PrintStrUARTDebug("\033[2J\rUART0 Enabled\r\n");
 	volo_printPrompt(&sUART0Buff);
 	PrintStrUSBDebug("\033[2J\rUSB0 Enabled\r\n");

 	// Default terminal to print to on startup is the USB UART.
 	g_activeTermBuff = &sUART0Buff;

	// Initialize the calibration table from FLASH memory.
	Stel_InitFlashCalibrationTables();

	//TODO enable SSI1 as slave
	// This is meant to be a slave interface to the connected FPGA.

	//TODO enable ADCs 4, 5, 6, 7
	/**
	 * These ADC input pins are connected to the following signals:
	 *  ADC4 - PD3 - WiFi RX (from MAX2015)
	 *  ADC5 - PD2 - WiFi TX (directly from PA PDET)
	 *  ADC6 - PD1 - UHF TX (from MAX2014)
	 *  ADC7 - PD0 - UHF RX (from MAX2014)
	 **/

	// Enable RF Control lines: PA enablement and antenna switches are
	// controlled through PE4, PC[4-7]. All lines are pull-down to either
	// deactivate the PAs they are tied to or to default that ANT1 <=> TX
	// in the switch configuration.
	SetupRFCtrl();

	// Turn on LED indicators on PA7, and PA6 as of WSD Rev 3
	SetupLEDs();

	// Set radio state struct to rational defaults.
	Lime_InitRadioState();
	// Set all default values of radio
	Lime_XceiverPwrOnInitializationSequence();
	// Enables interrupts of PB0 (RxEn) and PB1 (TxEN) for FPGA control
	ROM_IntEnable(INT_GPIOB);

#ifdef __ENABLE_UART__
	// we don't reset the terminal when a USB device is attached because it
	// can't open a terminal until it enumerates, and when it enumerates,
	// THEN we reset the terminal.
	g_cResetTerminalPendingFlag = 1;
#endif //__ENABLE_UART__

#ifdef __DEBUG_UART__
	PrintStrUARTDebug("Entering Main Loop...\r\n");
#endif //__DEBUG_UART__
	//============================================ MAIN LOOP
	while(1)
	{
		// sit and wait for interrupt-driven events to occur
		// one of these events could be the establishment of a USB terminal
		// in that case, we want to print our welcome splash screen, but ONLY
		// once. The reason this is done here and not in the ISR is that the
		// USB buffers don't empty while handling an ISR.
		if (g_cResetTerminalPendingFlag)
		{
			g_cResetTerminalPendingFlag = 0;
			Stel_ResetTerminalDevice();
		}

		// check to see if the UART0 peripheral has recently received chars needing
		// processing
		if (iUART0BufferHasChars)
		{
			iUART0BufferHasChars = 0;
			// process bytes coming from the UART interface
			Stel_ProcessUART0Input();

		}

		// check to see if the USB0 peripheral has recently received chars needing
		// processing
		if (iUSB0BufferHasChars)
		{
			iUSB0BufferHasChars = 0;
			// process bytes coming from the USB interface
			Stel_ProcessUSB0Input();
		}

		// Heartbeat
		g_iMainPollLoopCounter++;
		if (g_iMainPollLoopCounter > WSD_LED_BLINK_SPEED)
		{
			g_iMainPollLoopCounter = 0;
			Stel_ToggleHeartBeatLED();
		}
	}
	//============================================ END MAIN LOOP
}

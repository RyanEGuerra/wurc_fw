/*
 * wsd_ssi_master.c
 *
 * Contains all helper functions for 4-pin serial interface. This code acts as master
 * to a connected slave Lime Microsystems LMS6002D, and provides many library functions
 * and macros to calibrate and set parameters for the LMS6002D. Some day, this will be
 * a standalone library that is platform-independant, but that is not today.
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VOLO WIRELESS,LLC SHALL NOT,
 *   UNDER ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  Created on: Feb 28, 2013
 *  Edited: 	July 16, 2013
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 *      Author: Narendra Anand (nanand@rice.edu)
 *      Thanks: Holly Liang
 *
 *  Initial calibration algorithm work is thanks to Narendra, and a big thanks to Holly for
 *  initial testing with the Lime Dev Board.
 */

#include "include/wsd_lms6002d_lib.h"

// A phantom variable to load data to that we want to disappear.
// Declared since Tx/Rx switching uses this variable and we don't
// want to keep allocating/de-allocating from the heap.
unsigned long g_ulDevNull;

// Keep track of all RF parameters that various settings depend or switch off of.
RadioState_t trx;

//*****************************************************************************
// Initialize radio state struct to good default values. This should be one of
// the first things you call in main().
//*****************************************************************************
void
Lime_InitRadioState(void)
{
	trx.ulCurrentRxFreq = 0;
	trx.ulCurrentTxFreq = 0;
	trx.ulCurrentTxGain = 0;
	trx.ulCurrentRx_I_DCO = 0;
	trx.ulCurrentRx_Q_DCO = 0;
	trx.ulRxFREQSEL = 0;
	trx.ulTxRxPAOffTimeout = 0;
	trx.ulRxTxPLLSettlingTimeout = 0;
	//TxRxSWMode_FDD, TxRxSWMode_TDD
	trx.TxRxSwitchingMode = TxRxSWMode_FDD;
	//LMS_CAL_NORMAL, LMS_CAL_RF_LOOPBACK, LMS_CAL_BB_LOOPBACK
	trx.CalibrationMode = LMS_CAL_NORMAL;
	//EXTERNAL_LNA, INTERNAL_LNA
#if defined(__VOLO_WURC_REV_RED__) || defined(__VOLO_WURC_REV_GREEN__) || defined(__VOLO_WURC_REV_BLUE__)
	trx.RxInputMode = INTERNAL_LNA;
#endif // RED, GREEN, BLUE
	trx.verboseOutputEnabled = TRUE;
	trx.automaticCalLoadingEnabled = TRUE;
	trx.automaticTxRxSwitchingEnabled = FALSE;
	trx.isTransmittingInUHFBand = TRUE;  // NOTE: this is set up right now for Tx/Rx in half-duplex.
	trx.isReceivingInUHFBand = TRUE;
	trx.flashCommitHasBeenVerified = FALSE;
	trx.directCTRL_LNAIsActive = TRUE;
	trx.txFSync = FALSE;
	trx.rxFSync = FALSE;
	trx.txIQInterleave = FALSE;
	trx.rxIQInterleave = FALSE;
	trx.dacClkEdgePolarity = TRUE;
	trx.adcPhaseSel = FALSE;
	trx.clkNonOverlapAdj = 0;
#if defined(__VOLO_WURC_REV_RED__) || defined(__VOLO_WURC_REV_GREEN__) || defined(__VOLO_WURC_REV_BLUE__)
	trx.TRXState = RX_NORMAL;
#endif // RED, GREEN, BLUE
}

//*****************************************************************************
//
// Slow the SSI0 interface source clock to [WSD_SLOW_SSI_RATE] Hz given an
// 80 MHz SysCLK.

// The Stellaris line has a design defect insofar as it can WRITE via the SSI
// master interface at 40 MHz, but it cannot properly READ data back at a speed
// much faster than 10 MHz. It'll complete the operation just fine, but the
// read bytes in the RX buffer will be bit shifted and the MSB will be lost.
//
// Yeah, the datasheet is errata'd to let people know this. Great.
//
// A good value for WSD_SLOW_SSI_RATE is 10 MHz.
//
//*****************************************************************************
void
Stel_SetSSI0ClkSlow(void)
{
	// Reconfigure the Synchronous serial interface - base addr, src clk rate,
	// 									  protocol, mode, bit rate, data width
	SSIConfigSetExpClk(SSI0_BASE, WSD_SYS_CLK_RATE, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER,
			WSD_SLOW_SSI_RATE, 16);
	// re-enable the SSI interface
	SSIEnable(SSI0_BASE);
//	isFastSerial0 = 0;
}

//*****************************************************************************
//
// Speed the SSI0 interface source clock to [WSD_FAST_SSI_RATE] Hz given an
// 80 MHz SysCLK.

// The Stellaris line has a design defect insofar as it can WRITE via the SSI
// master interface at 40 MHz, but it cannot properly READ data back at a speed
// much faster than 10 MHz. It'll complete the operation just fine, but the
// read bytes in the RX buffer will be bit shifted and the MSB will be lost.
//
// Yeah, the datasheet is errata'd to let people know this. Great.
//
// A good value for WSD_FAST_SSI_RATE is 40 MHz. The maximum SSI TX bit rate in
// master mode is: SysCLK/2.
//
//*****************************************************************************
void
Stel_SetSSI0ClkFast(void)
{
	// Reconfigure the Synchronous serial interface - base addr, src clk rate,
	// 									  protocol, mode, bit rate, data width
	SSIConfigSetExpClk(SSI0_BASE, WSD_SYS_CLK_RATE, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER,
			WSD_FAST_SSI_RATE, 16);
	// re-enable the SSI interface
	SSIEnable(SSI0_BASE);
	// keep track of the speed of the SSI clock
//	isFastSerial0 = 1;
}

//*****************************************************************************
//
// Spends the minimum amount of time writing to the Lime Transceiver.
// There is no error checking or clock-setting--it is assumed that the clock
// is already set at maximum speed. There is no feedback to this function.
//
// \param ulAddr The 2-byte representation of the LMS6002D address to write to.
// \param ulData The 2-byte data to write to the configuration register addressed.
//
//*****************************************************************************
void
Lime_FastWrite(const unsigned long ulAddr, unsigned long ulData)
{
	unsigned long ulReadBuffer, ulTemp_1, ulTemp_2;

	//TODO Mask all interrupts before calling Write2LimeFast()


	// Format commands words for the SSI
	// See the LMS6002D data sheet for more information about this protocol
	ulData = ulData & LMS_DATA_MASK;
	ulTemp_2 = ulAddr & LMS_ADDR_MASK;
	ulTemp_2 = ulTemp_2  << 8;
	ulTemp_1 = ulTemp_2 | LMS_WRITE_CMD_MASK ;
	ulTemp_1 = ulTemp_1 + ulData;

	// Write the data to SSI interface
	SSIDataPutNonBlocking(SSI0_BASE, ulTemp_1);

	// Wait for SSI transmission to complete
	while (SSIBusy(SSI0_BASE)==true)
	{
		//block
	}

	// Get the first word from input FIFO buffer.
	// IMPORTANT: It is dropped because this function is assumed to be called
	// only when the SSI bit rate is WSD_FAST_SSI_RATE. In that case, any
	// received words are corrupt, but we still need to clear the buffer of the
	// corrupt word.
	//TODO - we should be able to somehow indicate to higher layers that the RX
	// buffer has a garbage frame without blocking. This would allow many
	// Fast Write operations in a row without having to block to clear the
	// buffers.
	// IMPORTANT: It is important to note that EVERY SSI transaction is two-way
	// in 4-wire mode. It doesn't matter if the slave sends nothing--a blank
	// word is still read into the SSI RX buffer.
	SSIDataGetNonBlocking(SSI0_BASE, &ulReadBuffer);

	// There is no error checking in this function in order to save time.
	return;
}

//*****************************************************************************
//
// Reads from the passed address of the LMS6002D into the passed data buffer.
// No other output is given.
//
// \param ulAddr The 2-byte representation of the LMS6002D address to write to.
// \param pulData A long pointer to which the read 2-byte data will be written.
//
//*****************************************************************************
void
Lime_Read(unsigned long ulAddr, unsigned long* pulData)
{
	// Format SSI command words
	ulAddr = ulAddr & LMS_ADDR_MASK;
	ulAddr = ulAddr<<8;

	// Make sure the SSI clock is slow, or else the read word will be corrupt.
	//TODO -
	Stel_SetSSI0ClkSlow();

	// Initiate the read command by pushing the command to the SSI TX buffer
	SSIDataPutNonBlocking(SSI0_BASE, ulAddr);

	// Wait for SSI transmission to complete
	while (SSIBusy(SSI0_BASE)==true);

	// Pull the read results from the SSI0 RX buffer
	SSIDataGet(SSI0_BASE, pulData);

	// When done, reset the SSI clock to full-speed.
	Stel_SetSSI0ClkFast();
}

//*****************************************************************************
//
// Writes the passed data to the given register in the LMS6002D with readback.
// Validates codewords, etc... and returns a 1 if an error occured.
//
// \param ulAddr The 2-byte representation of the LMS6002D address to write to.
// \param ulData The 2-byte data to write to the configuration register addressed.
//
// \returns a 1 if an error in the write/readback occurs, else a 0.
//
//*****************************************************************************
int
Lime_RobustWrite(const unsigned long ulAddr, const unsigned long ulData)
{
	unsigned long ulTest;
	int is_error = 0;

	// This is super-robust, so check to make sure that the data is formatted
	// correctly.
	if ((ulData & LMS_DATA_MASK) != ulData)
	{
		char pcBuff[4];
		is_error += 1;
		Stel_ThrowError("Masked Extra Bits");
		Stel_PrintReg(ulAddr, pcBuff);
		PrintStr(", ");
		Stel_PrintReg(ulData, pcBuff);
		__newline
	}

	// run at the slow clock speed.
	Stel_SetSSI0ClkSlow();

	// WriteToLimeFast doesn't reset the clock speed because it assumes
	// it's at the fastest speed already. So this actually executes at
	// low speed.
	Lime_FastWrite(ulAddr, ulData);

	// Readback the value that was just written.
	Lime_Read(ulAddr, &ulTest);

	// there is an error if the value isn't read properly.
	if (ulTest != ulData)
	{
//		char cData[4];
//		char cTest[4];
		char pcBuff[4];
		is_error += 1;

		Stel_ThrowError("Readback Fail");
		Stel_PrintReg(ulAddr, pcBuff);
		PrintStr(", ");
		Stel_PrintReg(ulData, pcBuff);
		PrintStr(", ");
		Stel_PrintReg(ulTest, pcBuff);
		PrintStr(", ");
		__newline

//		ltoa(ulData, cData);
//		ltoa(ulTest, cTest);
//		USB0PrintStr("! Readback data does not match: IN [");
//		USB0PrintStr((unsigned char*)cData);
//		USB0PrintStr("] OUT [");
//		USB0PrintStr((unsigned char*)cTest);
//		USB0PrintStr("]");
//		__newline
	}

	// When done, reset the SSI clock to full-speed.
	Stel_SetSSI0ClkFast();
	return is_error;
}

//*****************************************************************************
//
// Writes the passed data to the given register in the LMS6002D with readback.
// Validates codewords, etc... and returns a 1 if an error occured.
//
// This version of the above function does NOT print anything else to the
// terminal in order that you don't disrupt machine output (JSON, inparticular)
//
// \param ulAddr The 2-byte representation of the LMS6002D address to write to.
// \param ulData The 2-byte data to write to the configuration register addressed.
//
// \returns a 1 if an error in the write/readback occurs, else a 0.
//
//*****************************************************************************
int
Lime_RobustWriteTest(const unsigned long ulAddr, const unsigned long ulData)
{
	unsigned long ulTest;
	int is_error = 0;

	// This is super-robust, so check to make sure that the data is formatted
	// correctly.
	if ((ulData & LMS_DATA_MASK) != ulData)
	{
		is_error += 1;
	}

	// run at the slow clock speed.
	Stel_SetSSI0ClkSlow();

	// WriteToLimeFast doesn't reset the clock speed because it assumes
	// it's at the fastest speed already. So this actually executes at
	// low speed.
	Lime_FastWrite(ulAddr, ulData);

	// Readback the value that was just written.
	Lime_Read(ulAddr, &ulTest);

	// there is an error if the value isn't read properly.
	if (ulTest != ulData)
	{
		is_error += 1;
	}

	// When done, reset the SSI clock to full-speed.
	Stel_SetSSI0ClkFast();
	return is_error;
}

//*****************************************************************************
//
// Checks to see if the LMS6002D is connected & serial communications work.
// This is accomplished by writing to one of the spare registers on the LMS6002D
// and reading back the written value with the RobustWrite function.
//
// \returns a 1 if an error occurred
//
//*****************************************************************************
int
Lime_ConnectionLost(void)
{
	int iErrorsOccurred = 0;
	unsigned long ulTestVal;

	ulTestVal = 0xAC; //10101100
	iErrorsOccurred += Lime_RobustWriteTest(LMS_SPARE_CONFIG_R_1, ulTestVal);
	ulTestVal = 0x35; //00110101
	iErrorsOccurred += Lime_RobustWriteTest(LMS_SPARE_CONFIG_R_1, ulTestVal);

	return iErrorsOccurred;
}

//*****************************************************************************
//
// Prints out information about the Lime LMS6002D transceiver chip currently
// connected via the SSI0 bus. Checks to see if the chip is actually connected.
//
// \returns a 1 upon failure, 0 upon successful read
//
//*****************************************************************************
int
Lime_GetChipInformation(void)
{
	unsigned long ulVersion, ulRevision;
	char cVersion[4];
	char cRevision[4];

	// remember: the read function can't flag an error
	Lime_Read(LMS_CHIP_INFO_R, &ulVersion);
	// mask out revision and version values
	ulRevision = ulVersion & LMS_REVISION_MASK;
	ulVersion = (ulVersion & LMS_VERSION_MASK) >> 4;
	// convert to strings and print to terminal
	PrintStr("LMS6002D Silicon Version: ");
	ltoa(ulVersion, cVersion);
	ltoa(ulRevision, cRevision);
	PrintStr((unsigned char*)cVersion);
	PrintStr(".");
	PrintStr((unsigned char*)cRevision);
	__newline
	return Lime_ConnectionLost();
}

//*****************************************************************************
//
// LMS6002D DCOCal that implements augmentation in "023a_FAQ_v1.0r8.pdf" #4.7
// See Lime_GeneralDCCalibrationSubroutine() comments for more details.
//
// \param ulBase The LMS6002D memory map base address for the calibration module
// \param ulDCAddr The specific DC_ADDR address of the DCO calibration module
//
// \return 0 on success, 1 if the algorithm failed to "converge."
//
//*****************************************************************************
//int
//Lime_GeneralDCCalibration_OLD(const unsigned long ulBase, const unsigned long ulDCAddr)
//{
//	unsigned const long LOCAL_DCO_RESULT_REG = ulBase + LMS_DCO_RESULT_OFFSET;
////	unsigned const long LOCAL_DCO_STATUS_REG = ulBase + LMS_DCO_STATUS_OFFSET;
//	unsigned const long LOCAL_DCO_CNTVAL_REG = ulBase + LMS_DCO_CNTVAL_OFFSET;
//	unsigned const long LOCAL_DCO_CLBR_REG = ulBase + LMS_DCO_CALIB_OFFSET;
//	unsigned long ulBuff;
//	int errorsOccurred = 0;
//
//	// Run basic DCO calibration algorithm
//	errorsOccurred +=
//			Lime_GeneralDCCalibrationSubroutine(ulBase, ulDCAddr);
//
//	// check DCO setting value for valid setting, since DC_LOCK is apparently not
//	// a reliable indicator.
//	Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
//	if (ulBuff != LMS_DCO_RESULT_INVALID_HI)
//	{
//		// great! Done!
//		return errorsOccurred;
//	}
//	else
//	{
//		// debug output
//		if (trx.verboseOutputEnabled)
//		{
//			PrintStr("Invalid DCOCal, repeat");
//			__newline
//		}
//
//		// We need to try DCO calibration once again, but with a different initial
//		// condition. We'll load a 0 into the DC_CNTVAL setting register
//		errorsOccurred +=
//				Lime_SetRegisterField(LOCAL_DCO_CNTVAL_REG, 0x00, LMS_DCO_CNTVAL_M);
//
//		// Toggle DC_LOAD to load the value in DC_CNTVAL
//		errorsOccurred +=
//				Lime_SetRegisterField(LOCAL_DCO_CLBR_REG, 0xFF, LMS_DC_CALIB_LOAD);
//		errorsOccurred +=
//				Lime_SetRegisterField(LOCAL_DCO_CLBR_REG, 0x00, LMS_DC_CALIB_LOAD);
//
//		// Run basic DCO calibration algorithm again; this should work.
//		errorsOccurred +=
//					Lime_GeneralDCCalibrationSubroutine(ulBase, ulDCAddr);
//
//		// Check value against lower bound
//		Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
//		if (ulBuff != LMS_DCO_RESULT_INVALID_LO)
//		{
//			// Good to go
//			return errorsOccurred;
//		}
//		else
//		{
//			// Lime says this is bad: it should never happen because QC specifically
//			// catches this condition (i.e. the manufacturing tolerance is beyond
//			// compensation with on-chip resources.)
//			if (trx.verboseOutputEnabled)
//			{
//				// We don't always throw this because the first calibration iter
//				// always sucks for some reason.
//				Stel_ThrowError("Invalid DC Cal Result");
//			}
//			return ++errorsOccurred;
//		}
//
//	}
//}

//*****************************************************************************
//
// Implementation of the LMS6002D "General DC Calibration Procedure."
// This is given in Figure 4.1 on page 36 of the Programming & Calibration Guide
//
// Briefly, the function accepts a calibration module base address and a specific
// calibration module address, triggers a DC calibration on that module, and then
// waits until the module signals that the calibration is complete.
//
// Constants are provided in wsd_defines.h to make your life easier.
//
// \param ulBase The LMS6002D memory map base address for the calibration module
// \param ulDCAddr The specific DC_ADDR address of the DCO calibration module
//
// \return 0 on success, 1 if the algorithm failed to "converge."
//
//*****************************************************************************
//int
//Lime_GeneralDCCalibrationSubroutine(const unsigned long ulBase, const unsigned long ulDCAddr)
//{
//	unsigned const long LOCAL_DCO_CLBR_REG = ulBase + LMS_DCO_CALIB_OFFSET;
//	unsigned const long LOCAL_DCO_STATUS_REG = ulBase + LMS_DCO_STATUS_OFFSET;
//	unsigned const long LOCAL_DCO_RESULT_REG = ulBase + LMS_DCO_RESULT_OFFSET;
//	unsigned long ulBuff;
//	int try_count;
//	int errorOccurred = 0;
//	char pcBuff[10];
//
//	// Set the DC_ADDR field in the DCO calibration register
//	// RYAN - 06182013 swapped back to hardcoded
//	Lime_Read(LOCAL_DCO_CLBR_REG, &ulBuff);
//	ulBuff &= ~LMS_DC_CALIB_ADDR; 	// mask out non-DC_ADDR fields
//	ulBuff |= ulDCAddr;				// set with new DC_ADDR field
//	Lime_FastWrite(LOCAL_DCO_CLBR_REG, ulBuff);
//
////	errorOccurred +=
////			Lime_SetRegisterField(LOCAL_DCO_CLBR_REG, ulDCAddr, LMS_DC_CALIB_ADDR);
//
//	// Pulse the START_CLBR bit
//	Lime_Read(LOCAL_DCO_CLBR_REG, &ulBuff);
//	ulBuff |= LMS_DC_CALIB_START_CLBR;	// set bit
//	Lime_FastWrite(LOCAL_DCO_CLBR_REG, ulBuff);
//	ulBuff &= ~LMS_DC_CALIB_START_CLBR;	// clr bit
//	Lime_FastWrite(LOCAL_DCO_CLBR_REG, ulBuff);
//
//
//	for (try_count = 0; try_count < MAXIMUM_DCO_CLBR_TRY_COUNT; try_count++)
//	{
//		// wait N microseconds; the manual says 6.4 us, but we don't know if
//		// that takes into account serial xmission speeds or what. Twiddle is
//		// available as a #define in wsd_settings.h
//		__NOP_DELAY_LOOP(DCO_CLBR_WAIT_CYCLES);
//
//		// check DC_CLBR_DONE status
//		Lime_Read(LOCAL_DCO_STATUS_REG, &ulBuff);
//		// is busy status is high, then we're not done yet and loop again
//		if ((ulBuff & LMS_DC_STATUS_CLBR_BUSY_M) == 0)
//		{
//			//debug
////			PrintStr("DC_LOCK: ");
////			myLong2Hex((ulBuff & LMS_DC_STATUS_LOCK_M), pcBuff, 2);
////			PrintStr((unsigned char *)pcBuff);
////			PrintStr(", DC_UD: ");
////			myLong2Hex(ulBuff & LMS_DC_STATUS_UD_M, pcBuff, 1);
////			PrintStr((unsigned char *)pcBuff);
////			Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
////			myLong2Hex((unsigned long)ulBuff, pcBuff, 2);
////			PrintStr((unsigned char *)pcBuff);
////			__newline
//
//			// if DC_LOCK is 000 or 111, then we're done!
//			if ( ((ulBuff & LMS_DC_STATUS_LOCK_M) == LMS_DC_STATUS_LOCK_M) ||
//				 ((ulBuff & LMS_DC_STATUS_LOCK_M) == 0) )
//			{
//				// success!
//
//				// Some debug output.
//				if (trx.verboseOutputEnabled)
//				{
//					PrintStr("DC Cal of 0x");
//					myLong2Hex(ulBase, pcBuff, 2);
//					PrintStr((unsigned char *)pcBuff);
//					PrintStr(", ADDR: 0x");
//					myLong2Hex(ulDCAddr, pcBuff, 2);
//					PrintStr((unsigned char *)pcBuff);
//					PrintStr(", try_cnt: ");
//					myLong2Hex((unsigned long)try_count, pcBuff, 2);
//					PrintStr((unsigned char *)pcBuff);
//					PrintStr(", regval: ");
//					Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
//					myLong2Hex((unsigned long)ulBuff, pcBuff, 2);
//					PrintStr((unsigned char *)pcBuff);
//					__newline
//				}
//
//				return errorOccurred;
//			} //end if lock status == 1 or 0
//		} //((ulBuff & LMS_DC_STATUS_CLBR_BUSY_M) == 0)
//	} //(try_count = 0; try_count < MAXIMUM_DCO_CLBR_TRY_COUNT; try_count++)
//
//	// failure!
//	if (trx.verboseOutputEnabled)
//	{
//		PrintStr("DC Cal of 0x");
//		myLong2Hex(ulBase, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(", ADDR: 0x");
//		myLong2Hex(ulDCAddr, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(", try_cnt: ");
//		myLong2Hex((unsigned long)try_count, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(", regval: ");
//		Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
//		myLong2Hex((unsigned long)ulBuff, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(" - Inferred Lock");
//		__newline
//	}
//
//	return errorOccurred;
//}

//*****************************************************************************
//FIXME
//*****************************************************************************
int
Lime_GeneralDCCalibration(const unsigned long ulBase, const unsigned long ulDCAddr)
{
	unsigned const long LOCAL_DCO_CLBR_REG = ulBase + LMS_DCO_CALIB_OFFSET;
	unsigned const long LOCAL_DCO_RESULT_REG = ulBase + LMS_DCO_RESULT_OFFSET;
	unsigned long ulBuff;
	char pcBuff[10];

	int errors = 0;

	// Set the DC_ADDR register of the target DC calibration block. This selects
	// the module that we're about to calibrate.
	errors += Lime_SetRegisterField(LOCAL_DCO_CLBR_REG, ulDCAddr, LMS_DC_ADDR_M);

	// ADDED BY REG - this is NOT in the algorithm description...
	// This is the default value of this register, but it seems that the new
	// calibration algorithm starts with the assumption that this is the starting
	// value. Let's make sure that it is just for completeness.
//	errors += Lime_SetRegisterField(LOCAL_DCO_RESULT_REG, 31, LMS_DCO_RESULT_M);

	// Pulse the Start Calibration Bit. This sequency is WAAAAY more safe and
	// slower than it needs to be, but let's be sure nothing bad is happening.
	errors += Lime_SetRegisterField(LOCAL_DCO_CLBR_REG,
									LMS_DC_START_CLBR_M,
									LMS_DC_START_CLBR_M);
	errors += Lime_SetRegisterField(LOCAL_DCO_CLBR_REG,
									0x00,
									LMS_DC_START_CLBR_M);

	// Get the output calibration value. If it's no 31, then we assume DC calibration
	// was good. If not, we try again.
	Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
	if (ulBuff & LMS_DCO_RESULT_M == 31)
	{
		// Set the output register to value 0x00, to try achieving calibration from
		// the other direction.
		errors += Lime_SetRegisterField(LOCAL_DCO_RESULT_REG, 0x00, LMS_DCO_RESULT_M);

		// Pulse the Start Calibration Bit. This sequency is WAAAAY more safe and
		// slower than it needs to be, but let's be sure nothing bad is happening.
		errors += Lime_SetRegisterField(LOCAL_DCO_CLBR_REG,
										LMS_DC_START_CLBR_M,
										LMS_DC_START_CLBR_M);
		errors += Lime_SetRegisterField(LOCAL_DCO_CLBR_REG,
										0x00,
										LMS_DC_START_CLBR_M);

		// Check the resulting value; if it's still zero, then DC calibration
		// failed. This shouldn't happen as long as everything is working well.
		Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
		if (ulBuff & LMS_DCO_RESULT_M == 0)
		{
			// BAD!
			PrintStr("DC Cal of 0x");
			myLong2Hex(ulBase, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(", ADDR: 0x");
			myLong2Hex(ulDCAddr, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			Stel_ThrowError("DCO Cal Lock FAILED");
			return 1;
		}
	}

	// Some debug output.
	if (trx.verboseOutputEnabled)
	{
		PrintStr("DC Cal of 0x");
		myLong2Hex(ulBase, pcBuff, 2);
		PrintStr((unsigned char *)pcBuff);
		PrintStr(", ADDR: 0x");
		myLong2Hex(ulDCAddr, pcBuff, 2);
		PrintStr((unsigned char *)pcBuff);
		PrintStr(", regval: ");
		Lime_Read(LOCAL_DCO_RESULT_REG, &ulBuff);
		myLong2Hex((unsigned long)ulBuff, pcBuff, 2);
		PrintStr((unsigned char *)pcBuff);
		__newline
	}

	return errors;
}

//*****************************************************************************
//
// Implementation of the LMS6002D "DC Offset Calibration of the LPF Tuning Module."
// From figure 4.2 on pg. 37 of the Programming & Calibration Guide.
//
// As far as I can tell, the algorithm gets DCO calibration values from a single
// top-level module and writes that value to the two LPF modules. This seems to
// be distinct from the individual I/Q DCO calibration modules in each LPF.
//
// Constants are provided in wsd_defines.h to make your life easier.
//
// \return 0 upon success. Returns a 1 if an error occurs or DC CAL doesn't converge
//
//*****************************************************************************
int
Lime_LPFCoreDCOffsetCalibration(void)
{
	unsigned long ulBuff, ulCalResult;
	unsigned long clken_5WasActive = 0;
	int iErrorOccurred = 0;

	// save the LPF CAL clock enable state, then set HIGH to enable
	Lime_Read(LMS_CLK_CTRL_R, &ulBuff);
	clken_5WasActive = ulBuff & LMS_CLK_EN_LPFCAL;
	Lime_SetClockCtrl(LMS_CLK_EN_LPFCAL, LMS_SET);

//	ulBuff |= LMS_CLK_EN_LPFCAL;
//	Lime_FastWrite(LMS_CLK_CTRL_R, ulBuff);

	// We discovered that the Lime GUI checks to see if the DC_SRESET bit is set in
	// LMS_TOP_BASE + LMS_DC_CALIB_BASEOFFSET. This is not in the flow graph, so lets
	// check to see if this ever happens; LMS_DC_CALIB_SRESET is a mis-nomer, it is an
	// active-low reset, so we WANT it to be high.
	Lime_Read(LMS_TOP_BASE + LMS_DCO_CALIB_OFFSET, &ulBuff);
	if (!(ulBuff & LMS_DC_CALIB_SRESET))
	{
		// bad; let's say something
		PrintStr("Setting DC_SRESET (2)");
		__newline
		Lime_SetRegisterField(LMS_TOP_BASE + LMS_DCO_CALIB_OFFSET,
							  0xFF,
							  LMS_DC_CALIB_SRESET);
	}

	// Run the general DC calibration procedure for the LPF Tuning Module
	if (Lime_GeneralDCCalibration(LMS_TOP_BASE, LMS_DC_LPF_TUNE_A))
	{
		// Lime_GeneralDCCalibration returns non-zero on ERROR
		iErrorOccurred += 1;
	}

	// get the DC Cal result value from DC_REGVAL
	Lime_Read(LMS_TOP_BASE + LMS_DCO_RESULT_OFFSET, &ulCalResult);
	ulCalResult &= LMS_DC_RESULT_REGVAL_M;

	// Set the DCO_DACCAL value for the RX LPF
	Lime_Read(LMS_RXLPF_BASE + LMS_DCO_DACCAL_OFFSET, &ulBuff);
	ulBuff &= ~LMS_DCO_DACCAL_M;	//clear RXLPF DACCAL
	ulBuff |= ulCalResult;			//set DACCAL
	iErrorOccurred +=
			Lime_RobustWrite(LMS_RXLPF_BASE + LMS_DCO_DACCAL_OFFSET, ulBuff);

	// set the DCO_DACCAL value for the TX LPF
	Lime_Read(LMS_TXLPF_BASE + LMS_DCO_DACCAL_OFFSET, &ulBuff);
	ulBuff &= ~LMS_DCO_DACCAL_M;	//clear RXLPF DACCAL
	ulBuff |= ulCalResult;			//set DACCAL
	iErrorOccurred +=
			Lime_RobustWrite(LMS_TXLPF_BASE + LMS_DCO_DACCAL_OFFSET, ulBuff);

	// restore the LPF CAL clock enable state to what it was before
	if (clken_5WasActive)
	{
		// don't do anything--it's already active
	}
	else
	{
		iErrorOccurred +=
				Lime_SetClockCtrl(LMS_CLK_EN_LPFCAL, LMS_CLEAR);
	}

	return iErrorOccurred;
}

//*****************************************************************************
//
// Implementation of the LMS6002D "TX/RX LPF DC Offset Calibration" routine.
// From Figure 4.3 on pg. 38 of the Programming & Calibration Guide.
//
// As far as I can tell, this performs the DCO calibration the way I expect:
// enable and the trigger the module and then check the result. The only tricky
// part is the addresses are different for the TX and RX directions, hence they
// are arguments to this function.
//
// To prevent errors, go ahead and copy/paste the following invocations:
//
// Lime_TXRXLPFDCOffsetCalibration(LMS_TXLPF_BASE, LMS_CLK_EN_TX_LPF_DCCAL_M);
// Lime_TXRXLPFDCOffsetCalibration(LMS_RXLPF_BASE, LMS_CLK_EN_RX_LPF_DCCAL_M);
//
//TODO - FAQ 5.26 suggests powering down the DC comparator for increased RX linearity
//       The register definitions are not clear since they seem to be unused or
//       conflicting. We should look at this.
//
// \return 0 upon success. Returns a 1 if an error occurs or DC CAL doesn't converge
//
//*****************************************************************************
int
Lime_TXRXLPFDCOffsetCalibration(unsigned long ulBase, unsigned long ulCLKENMask)
{
	unsigned long ulBuff, ulSavedCLKEN;
	int errors = 0;

	// save the TX/RX LPF DCCAL clock enable state, then set HIGH to enable
	Lime_Read(LMS_CLK_CTRL_R, &ulBuff);
	ulSavedCLKEN = ulBuff & ulCLKENMask;
	errors +=
			Lime_SetClockCtrl(ulCLKENMask, LMS_SET);

	// enable the DC offset comparator for the LPF; 0 = powered up
	// NOTE: this reference has been removed from recent calibration revisions.
	errors += Lime_SetRegisterField(ulBase + LMS_DCOCMP_PD_OFFSET,
								    0x00, LMS_DCOCOM_LPF_PD_M);

	// Run the general DC calibration procedure for the (I) TX/RX LPF Module
	errors +=
			Lime_GeneralDCCalibration(ulBase, LMS_DC_TXRXLPF_I_A);

	// Run the general DC calibration procedure for the (Q) TX/RX LPF Module
	errors +=
			Lime_GeneralDCCalibration(ulBase, LMS_DC_TXRXLPF_Q_A);

	// restore the LPF CAL clock enable state
	errors +=
			Lime_SetRegisterField(LMS_CLK_CTRL_R, ulSavedCLKEN, ulCLKENMask);

	// DISable the DC offset comparator for the LPF; 1 = powered down
	// The Lime Micro data sheet recommends turning this off after calibration
	errors +=
			Lime_SetRegisterField(ulBase + LMS_DCOCMP_PD_OFFSET,
								  0xFF, LMS_DCOCOM_LPF_PD_M);

	return errors;
}

//*****************************************************************************
//
// Implementation of the LMS6002D "RXVGA2 DC Offset Calibration" routine.
// From Figure 4.4 on pg. 39 of the Programming & Calibration Guide.
//
// Look at Figure 3.7 on pg.33 of that same manual to see what's going on.
// Essentially, all of the DCO Calibration blocks in the RX VGA2 chain are
// triggered in sequence.
//
//TODO - FAQ 5.26 suggests powering down the DC comparator for increased RX linearity
//       The register definitions are not clear since they seem to be unused or
//       conflicting. We should look at this.
//
// \return 0 upon success. Returns a 1 if an error occurs.
//
//*****************************************************************************
int
Lime_RXVGA2DCOffsetCalibration(void)
{
	unsigned long ulBuff, ulSavedCLKEN;
	int errorsOccurred = 0;

	// save the clock enable state, then set RX VGA2 DCCAL to enable
	Lime_Read(LMS_CLK_CTRL_R, &ulBuff);
	ulSavedCLKEN = ulBuff;
	errorsOccurred +=
			Lime_SetClockCtrl(LMS_CLK_EN_RX_VGA2_DCCAL, LMS_SET);

	// enable the DCO Comparators; 0 = enabled.
	errorsOccurred +=
			Lime_SetRegisterField(LMS_RXVGA2_PD_R, 0x00,
					LMS_VGA2A_PD_DCO_COMP_M | LMS_VGA2B_PD_DCO_COMP_M);

	// Run the general DC calibration procedure for the VGA2 DC Reference Module
	errorsOccurred +=
			Lime_GeneralDCCalibration(LMS_RXVGA2_BASE, LMS_DC_RXVGA_DC_REF_A);

	// Run the general DC calibration procedure for the VGA2A I Channel
	errorsOccurred +=
			Lime_GeneralDCCalibration(LMS_RXVGA2_BASE, LMS_DC_VGA2A_I_A);

	// Run the general DC calibration procedure for the VGA2A Q Channel
	errorsOccurred +=
			Lime_GeneralDCCalibration(LMS_RXVGA2_BASE, LMS_DC_VGA2A_Q_A);

	// Run the general DC calibration procedure for the VGA2B I Channel
	errorsOccurred +=
			Lime_GeneralDCCalibration(LMS_RXVGA2_BASE, LMS_DC_VGA2B_I_A);

	// Run the general DC calibration procedure for the VGA2B Q Channel
	errorsOccurred +=
			Lime_GeneralDCCalibration(LMS_RXVGA2_BASE, LMS_DC_VGA2B_Q_A);

	// restore the RX VGA2 DCCAL clock enable state
	errorsOccurred +=
			Lime_SetRegisterField(LMS_CLK_CTRL_R, ulSavedCLKEN, LMS_CLK_EN_RX_VGA2_DCCAL);

	// DISable the DCO Comparators; 1 = disabled.
		errorsOccurred +=
				Lime_SetRegisterField(LMS_RXVGA2_PD_R, 0xFF,
						LMS_VGA2A_PD_DCO_COMP_M | LMS_VGA2B_PD_DCO_COMP_M);

	return errorsOccurred;
}

//*****************************************************************************
//
// Implementation of the LMS6002D "LPF Bandwidth Tuning" routine.
// From Figure 4.5 on pg. 40 of the Programming & Calibration Guide.
//
// There is actually a decent amount of stuff going on in the subroutine:
// The Top-Level RCCAL module is triggered, and its resulting value is
// written to the RCCAL settings on the RX and TX LPF modules.
//
// Note that this subroutine currently requires a 40 MHZ PLL reference clock
// to work. There is a technique for self-generating a reference clock when
// 40 MHz is not available, but we haven't implemented it here because it
// seems to rely on too many magic numbers and we don't really understand it.
//
// \return 0 upon success. Returns a 1 if an error occurs.
//
//*****************************************************************************
int
Lime_LPFBandwidthTuning(void)
{
	//FIXME THis is a HACK
	int errors = 0;

	unsigned long ulBuff, ulRCCAL;
//	int errors = 0;
	char pcBuff[6];

	// If the PLL reference clock is not 40 MHz, then we have to calibrate the module
	// with an internally-generated reference clock. In the Lime GUI, this triggers
	// a flurry of commands that we don't know what to do with--there isn't that much
	// transparency with this. Luckily, we plan to provide a 40 MHz reference clock
	// for the radio daughter-card, so we don't have to worry about this.
	if (LMS_PLL_REF_CLK_RATE != 40000)
	{
		return 1;
	}

	// Set LPF bandwidth to an arbitrary value (2.5 MHz in example, 10 MHz for
	// our planned WARP-like OFDM system). Lime claims the actual BW when
	// tuning is arbitrary and should hold across all BW values.
	// 11/12/2013 - we know how trusting Lime works out. Fuck that. It's
	// now 20 MHz for our OFDM system.
	// 02/2014 - Modifying this again to match Lime's flow diagram exactly. We saw this
	// algorithm choose terrible values in the past...
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R, LMS_LPF_BW_10, LMS_LPFCAL_2_BWC_M);
//	Lime_Read(LMS_TOP_LPFCAL_2_R, &ulBuff);
//	ulBuff &= ~LMS_LPFCAL_2_BWC_M;		// clear BWC_LPFCAL
//	ulBuff |= LMS_LPF_BW_10;			// set BWC_LPFCAL
//	if (Lime_RobustWrite(LMS_TOP_LPFCAL_2_R, ulBuff))
//	{
//		// Lime_RobustWrite returns a 1 on error
//		errors = 1;
//	}

	// Enable the LPFCAL module by setting LPFCAL_EN_CAL high
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R, 0xFF, LMS_LPFCAL_2_EN_CAL_M);
//	ulBuff |= LMS_LPFCAL_2_EN_CAL_M;
//	if (Lime_RobustWrite(LMS_TOP_LPFCAL_2_R, ulBuff))
//	{
//		// Lime_RobustWrite returns a 1 on error
//		errors = 1;
//	}

	// Pulse the RST_CAL_LPFCAL bit
	// This pulse MUST last longer than 100ns. Luckily, the max SPI rate is 50 MHz
	// which has a period of 20 ns, and each LMS SPI command is at least 16 bits long.
	Lime_Read(LMS_TOP_LPFCAL_1_R, &ulBuff);
	ulBuff |= LMS_LPFCAL_1_RST_CAL_M;	// set reset high
	Lime_FastWrite(LMS_TOP_LPFCAL_1_R, ulBuff);
	ulBuff &= ~LMS_LPFCAL_1_RST_CAL_M;	// set reset low
	Lime_FastWrite(LMS_TOP_LPFCAL_1_R, ulBuff);

	// We discovered that the Lime GUI checks to see if the DC_SRESET bit is set in
	// LMS_TOP_BASE + LMS_DC_CALIB_BASEOFFSET. This is not in the flow graph, so lets
	// check to see if this ever happens; LMS_DC_CALIB_SRESET is a mis-nomer, it is an
	// active-low reset, so we WANT it to be high.
	Lime_Read(LMS_TOP_BASE + LMS_DCO_CALIB_OFFSET, &ulBuff);
	if (!(ulBuff & LMS_DC_CALIB_SRESET))
	{
		// bad; let's say something
		PrintStr("Setting DC_SRESET (1)");
		__newline
		Lime_SetRegisterField(LMS_TOP_BASE + LMS_DCO_CALIB_OFFSET,
							  0xFF,
							  LMS_DC_CALIB_SRESET);
	}

	// Get the RC Calibration result from the TOP_DC_STATUS register
	// Note that TOP_RCCAL_LPF is [7-5] while each RCCAL_LPF is [6-4]
	Lime_Read(LMS_TOP_BASE + LMS_DCO_STATUS_OFFSET, &ulBuff);
	ulRCCAL = (ulBuff & LMS_RCCAL_LPFCAL_M) >> 1;	// save RCCAL

	// Disable the TOP LPF_CAL block. This is not in the flow chart,
	// but the GUI does send this command and it makes sense to me.
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R, 0x00, LMS_LPFCAL_2_EN_CAL_M);
//	Lime_Read(LMS_TOP_LPFCAL_2_R, &ulBuff);
//	ulBuff &= ~LMS_LPFCAL_2_EN_CAL_M;	// clear EN_CAL_LPFCAL bit
//	Lime_FastWrite(LMS_TOP_LPFCAL_2_R, ulBuff);

	// Set the RCCAL_LPF field for the RX LPF module to
	// our saved RCCAL value
	errors += Lime_SetRegisterField(LMS_RXLPF_BASE + LMS_RCCAL_OFFSET,
									ulRCCAL,
									LMS_RCCAL_LPF_M);
//	Lime_Read(LMS_RXLPF_BASE + LMS_RCCAL_OFFSET, &ulBuff);
//	ulBuff &= ~LMS_RCCAL_LPF_M; 	// clear RCCAL_LPF
//	ulBuff |= ulRCCAL;				// set RCCAL_LPF
//	if (Lime_RobustWrite(LMS_RXLPF_BASE + LMS_RCCAL_OFFSET, ulBuff))
//	{
//		// Lime_RobustWrite returns a 1 on error
//		errors = 1;
//	}

	// Set the RCCAL_LPF field for the TX LPF module.
	Lime_Read(LMS_TXLPF_BASE + LMS_RCCAL_OFFSET, &ulBuff);
	ulBuff &= ~LMS_RCCAL_LPF_M; // clear DC_RegVl
	ulBuff |= ulRCCAL;
	if (Lime_RobustWrite(LMS_TXLPF_BASE + LMS_RCCAL_OFFSET, ulBuff))
	{
		// Lime_RobustWrite returns a 1 on error
		errors = 1;
	}

	PrintStr("OLD LPFCAL: ");
	Stel_PrintReg(ulRCCAL, pcBuff);
	__newline

	errors += Lime_SetRegisterField((LMS_TXLPF_BASE + LMS_RCCAL_OFFSET),
							 0x03 << 4,
							 LMS_RCCAL_LPF_M);
	errors += Lime_SetRegisterField((LMS_RXLPF_BASE + LMS_RCCAL_OFFSET),
							 0x03 << 4,
							 LMS_RCCAL_LPF_M);

	// unless something bad happened, we're done!
	return errors;
}

//*****************************************************************************
//
// Executes the complete Auto Calibration Sequence.
// Section 4.7 in the LMS6002D Programming & Calibration Guide.
//
// Due to a tricky error to track down, the VERY FIRST time this is run after a
// a hard transceiver reset produces error messages. Therefore, this suppresses
// all text output if this is the first time being run by using verboseOutput
// flag.
//
// Acceptable arguments for ulCalibrationMode are:
//
//	LMS_CAL_NORMAL
// 	LMS_CAL_RF_LOOPBACK
//	LMS_CAL_BB_LOOPBACK
//
// \param ulCalibrationMode Accepts an argument for setting certain autocal params.
//
// \return 0 upon success. Returns > 1 if an error occurs. Returns -1 if output suppressed
//
//*****************************************************************************
int
Lime_AutoCalibrationSequence(void)
{
	int errors = 0;
	unsigned long ulSavedRxVGA2Gain;
	unsigned long ulSaveBBTxRxSwitchState;
	unsigned long ulBuff;

	// An infinite loop exists if communications with the LMS fail.
	// Let's check here and avoid bothering if they have.
	if (Lime_ConnectionLost())
	{
		Stel_ThrowError("Lime SPI Connection Is Bad");
		return 1;
	}

	if (trx.verboseOutputEnabled)
	{
		PrintStr("=== Running Autocalibration Sequence ===");
		__newline
	}

	// Temporarily disable BB Tx/Rx switching control if it's active.
	ulSaveBBTxRxSwitchState = trx.automaticTxRxSwitchingEnabled;
	if (trx.automaticTxRxSwitchingEnabled)
	{
		WSD_DisableBBTxRxSwitchingCtrl();
	}

	// DISABLE DACS - Disable the DACs while undergoing autocalibration.
	// This command issues a command to the baseband to send zeros to the
	// DAC. This ensures there is no TX RF signal during autocal.
	errors += Lime_SetDACEnable(DISABLE);

	// DISABLE RF LOOPB - If we're in RF loopback mode, disable loopback
	// mode temporarily.
	// This ensures that no energy is input to the Rx chain at all.
	// Note: Lime doesn't say to do this, but they DO say to avoid any input
	// to the Tx and Rx chain. We're trying this out because we don't really
	// think they've thouroughly vetted their calibration process.
	if (trx.CalibrationMode == LMS_CAL_RF_LOOPBACK)
	{
		errors += Lime_SetRegisterField(LMS_LOOPBACK_CTRL_R,
									  	LMS_RF_LOOPB_DISABLED,
									  	LMS_LOOPBACK_LBRFEN_M);
		Lime_SetAUXPAState(AUXPA_OFF);
	}

	// DISCONNECT RF Input - depending on the current mode of RX input,
	// try to disconnect it
	if (trx.TRXState == RX_NORMAL)
	{
		Lime_SetRxInputPath(EXTERNAL_LNA);
	}
	else if (trx.TRXState == RX_BYPASS_INT_LNA)
	{
		Lime_SetRxInputPath(INTERNAL_LNA);
	}
	WSD_TurnOffUHFLNA();

	// DISABLE LNAS - Direct test mode disable all LNAs. This ensures that
	// there is no RF input to the Rx chain. Don't do this if in RF Loopback.
	// Note: in RF Loopback, this should already be deactivated. :/
//	if (trx.CalibrationMode != LMS_CAL_RF_LOOPBACK)
//	{
	errors += Lime_DirectDisableInternalLNAs();
//  }

	// Somehow the default values were being overwritten at some point,
	// so I now enforce this before running DCO. I also notice I get bad values
	// if this is not set. Not sure where that started happening, or if I
	// just noticed it and it was always happening.
	Lime_Read(LMS_TOP_BASE + LMS_DCO_CALIB_OFFSET, &ulBuff);
	if (!(ulBuff & LMS_DC_CALIB_SRESET))
	{
		// bad; let's say something
		PrintStr("Setting DC_SRESET (0)");
		__newline
		Lime_SetRegisterField(LMS_TOP_BASE + LMS_DCO_CALIB_OFFSET,
							  0xFF,
							  LMS_DC_CALIB_SRESET);
	}

	// Maximize VGA2 gains, but save the last set value for later restoration
	ulSavedRxVGA2Gain = Lime_GetRegisterFieldVal(LMS_GAIN_RXVGA2_R, LMS_GAIN_RXVGA2_M);
	errors += Lime_SetRXVGA2Gain(10);

	// LPF Core DCO Calibration
	if (Lime_LPFCoreDCOffsetCalibration())
	{
		errors++;
		if (trx.verboseOutputEnabled)
		{
			Stel_ThrowError("DCOCal LPF Core");
		}
	}

	// LPF Bandwidth Tuning - choose the optimal RC values for the LPF  frequency
	// response due to process variation.
//	if (Lime_LPFBandwidthTuning() || Lime_NewLPFBandwidthTuning(LMS_LPF_BW_3))
	if ( Lime_NewLPFBandwidthTuning(LMS_LPF_BW_5) )
	{
		errors++;
		if (trx.verboseOutputEnabled)
		{
			Stel_ThrowError("DCOCal LPF BW");
		}
	}

	// TX LPF DCO Calibration - tune the DCO DACs for the I/Q of The TX LPFs.
	errors += Lime_SetRegisterField(LMS_DCOCOM_TXLPF_R,
									0x00, LMS_DCOCOM_TXLPF_EN_M);
	if (Lime_TXRXLPFDCOffsetCalibration(LMS_TXLPF_BASE, LMS_CLK_EN_TX_LPF_DCCAL))
	{
		errors++;
		if (trx.verboseOutputEnabled)
		{
			Stel_ThrowError("DCOCal TxLPF");
		}
	}
	errors += Lime_SetRegisterField(LMS_DCOCOM_TXLPF_R,
									0xFF, LMS_DCOCOM_TXLPF_EN_M);

	// RX LPF DCO Calibration - tune the DCO DACs for the I/Q of The RX LPFs.
	errors += Lime_SetRegisterField(LMS_DCOCOM_RXLPF_R,
									0x00, LMS_DCOCOM_RXLPF_EN_M);
	if (Lime_TXRXLPFDCOffsetCalibration(LMS_RXLPF_BASE, LMS_CLK_EN_RX_LPF_DCCAL))
	{
		errors++;
		if (trx.verboseOutputEnabled)
		{
			Stel_ThrowError("DCOCal RxLPF DCO");
		}
	}
	errors += Lime_SetRegisterField(LMS_DCOCOM_RXLPF_R,
									0xFF, LMS_DCOCOM_RXLPF_EN_M);

	// Tune the DCO DACS for the RXVGA2A and RXVGA2B I/Q stages.
	errors += Lime_SetRegisterField(LMS_DCOCOM_RXVGA2_R,
									0x00,
									(LMS_DCOCOM_RXVGA2B_EN_M | LMS_DCOCOM_RXVGA2A_EN_M));
	if (Lime_RXVGA2DCOffsetCalibration())
	{
		errors++;
		if (trx.verboseOutputEnabled)
		{
			Stel_ThrowError("DCOCal VGA2");
		}
	}
	errors += Lime_SetRegisterField(LMS_DCOCOM_RXVGA2_R,
									0xFF, (LMS_DCOCOM_RXVGA2B_EN_M | LMS_DCOCOM_RXVGA2A_EN_M));

	// RECONNECT RX Input to the correct RX Inputs depending on TRX Mode
	if (trx.TRXState == RX_NORMAL)
	{
		Lime_SetRxInputPath(INTERNAL_LNA);
	}
	else if (trx.TRXState == RX_BYPASS_INT_LNA)
	{
		Lime_SetRxInputPath(EXTERNAL_LNA);
	}
	if (Lime_Rx_IsUHFBand() && trx.CalibrationMode != LMS_CAL_RF_LOOPBACK)
	{
		// Turn on the external UHF LNA again.
		WSD_TurnOnUHFLNA();
	}

	// Enable loopback mode once again, if we're currently in RF loopback mode.
	if (trx.CalibrationMode == LMS_CAL_RF_LOOPBACK)
	{
		// The LNAs are enabled by the Lime_SetRxInputPath() in some cases
		// This ensures that they are disabled again for RF Loopback.
		Lime_DirectDisableInternalLNAs();
		// The RF Loopback path is connected to one of the Rx Mixer buffers,
		// but the particular buffer chosen depends on the target frequency.
		// I guess it makes sense that we switch off the Rx frequency, but
		// for anything meaningful, both Tx and Rx frequency much be within
		// the same band.
		if (Lime_Rx_IsWiFiBand())
		{
			errors += Lime_SetRegisterField(LMS_LOOPBACK_CTRL_R,
											LMS_RF_LOOPB_LNA_2,
											LMS_LOOPBACK_LBRFEN_M);
		}
		else if (Lime_Rx_IsUHFBand())
		{
			errors += Lime_SetRegisterField(LMS_LOOPBACK_CTRL_R,
											LMS_RF_LOOPB_LNA_1,
											LMS_LOOPBACK_LBRFEN_M);
		}
		else
		{
			Stel_ThrowError("Unsupported Rx RFLB Freq");
		}

		// Turn on the auxiliary RX Loopback PA, which should have been disabled earlier.
		Lime_SetAUXPAState(AUXPA_ON);
	}

	// Re-enable the DAC/ADC modules now that calibration is complete.
	errors += Lime_SetDACEnable(ENABLE);

	// Restore the RxVGA2 gain value
	errors += Lime_SetRXVGA2Gain(ulSavedRxVGA2Gain);

	if (trx.verboseOutputEnabled)
	{
		PrintStr("=== End Autocalibration ===");
		__newline
	}

	// Restore the saved BB Tx/Rx switching ctrl state
	if (ulSaveBBTxRxSwitchState)
	{
		WSD_EnableBBTxRxSwitchingCtrl();
	}

	// Re-enable text output and re-run auto-calibration sequence internally
//	trx.verboseOutputEnabled = TRUE;
//	if (ulSpareRegState == 0x00)
//	{
//		// Set the spare configuration register
//		errors += Lime_RobustWrite(LMS_SPARE_CONFIG_R_2, 0xAC);
//
//		// The g_currentOperationalMode will determine how the AutoCalSeq
//		// is executed.
//		errors =
//				Lime_AutoCalibrationSequence();
//	}

	return errors;
}

//*****************************************************************************
//
// Implementation of the LMS6002D "VCOCAP Code Selection Algorithm."
// From Figure 4.6 on pg. 41 of the LMS6002D Programming & Calibration Guide.
// Also, more information is available in section 3.4.3 on pg. 29 of how the
// algorithm works.
//
// Essentially, we're looking for the VCOCAP value that sets VTUNE to 1.5V,
// so we search for the two indices where CH or CL transitions at 2.5V and 0.5V
// and set our VCOCAP value to correspond to the index between those two
// transition indices.
//
// This calibration sequence yields a different VCOCAP value for each center
// frequency, so should be re-run whenever the channel is changed. For this
// reason, I have written this function to use non-robust SPI write commands
// in order to switch channels quickly.
//
// \return 0 upon success. Returns a 1 if an error occurs.
//
//*****************************************************************************
int
Lime_VCOCAPCalibration(const unsigned long ulPLLBaseAddr)
{
	unsigned long ulBuff, ulVCOCAPUpper, ulCH, ulCL, ulIndexCL, ulIndexCH;
	unsigned long ulSavedClockCtrl;
	unsigned const long LMS_PLL_VCOCAP_R = ulPLLBaseAddr + LMS_PLL_VCOCAP_OFFSET;
	unsigned const long LMS_PLL_VTUNE_R = ulPLLBaseAddr + LMS_PLL_VTUNE_OFFSET;
	int errorsOccurred = 0;

	char pcBuff[20];

	// Make sure that the DSM_SPI clocks are set, otherwise any changes we make
	// to PLL configuration registers will never be committed from the shadow
	// registers! Save the current clock settings before changing.
	Lime_Read(LMS_CLK_CTRL_R , &ulSavedClockCtrl);
	errorsOccurred +=
			Lime_SetClockCtrl(LMS_CLK_EN_RX_DSM_SPI | LMS_CLK_EN_TX_DSM_SPI, LMS_SET);

	// Make sure that PD_VCOCOMP_SX is NOT enabled. (read: the comparators are
	// actually on.)
	__setRegField(ulPLLBaseAddr + LMS_PLL_VCOCOMP_PD_OFFSET, LMS_PLL_PD_VCOCOMP_SX_M, 0x00, ulBuff);
//	Lime_Read(ulPLLBaseAddr + LMS_PLL_VCOCOMP_PD_OFFSET, &ulBuff);
//	ulBuff &= ~LMS_PLL_PD_VCOCOMP_SX_M;		//turn off the power-down signal!
//	Lime_FastWrite(ulPLLBaseAddr + LMS_PLL_VCOCOMP_PD_OFFSET, ulBuff);

	// Set VCOCAP to 31, the middle of the range [0-63]
	Lime_Read(LMS_PLL_VCOCAP_R, &ulBuff);
	ulVCOCAPUpper = ulBuff & LMS_PLL_VOVCOREG_LSB_M;	// save the VOVCOREG LSB
	ulBuff &= ~LMS_PLL_VCOCAP_M; 						// clear VCOCAP field
	ulBuff |= 31;										// set VCOCAP field
	errorsOccurred +=
			Lime_RobustWrite(LMS_PLL_VCOCAP_R, ulBuff);

	// Wait some time. Twiddle is available as a #define in wsd_settings.h
	// Note: this was not specified by Lime MS, but we discovered that when
	// setting frequencies that were close to the current set frequency,
	// the VCOCAP algorithm would converge to the wrong value, mainly because
	// the first read of CH and CL would be wrong: (0, 0) rather than (1, 0),
	// say. This delay was tuned to be enough time to let the CH, CL values
	// to converge to the same, correct value no matter the target frequency.
	// It is not known why this is necessary.
	__NOP_DELAY_LOOP(LMS_VCO_SETTLING_CYCLES);

	// Check CH and CL. These two comparators indicate if VTUNE is respectively
	// greater than 2.5V or less than 0.5V, conditions in which PLL lock is not
	// guaranteed.
	Lime_Read(LMS_PLL_VTUNE_R, &ulBuff);
	ulCH = ulBuff & LMS_PLL_VTUNE_H_M;
	ulCL = ulBuff & LMS_PLL_VTUNE_L_M;
	// CH = CL = 1, ERROR!
	// This should not be possible. Something is bad.
	if (ulCH && ulCL)
	{
		return 1;
	}
	// CH = 1, CL = 0, we're to the left of the transition points
	// We will increment the VCOCAP index until we find both transition points
	else if (ulCH)
	{
		PrintStr("CH = 1, CL = 0");
		__newline

		ulIndexCH = 31;
		while (ulCH && (ulIndexCH < 63))
		{
			// Increment the VCOCAP index and check new ulCH value
			// also, don't forget we have to preserve the upper bits of
			// LMS_PLL_VCOCAP_R
			Lime_FastWrite(LMS_PLL_VCOCAP_R, ulVCOCAPUpper | ++ulIndexCH);
			// wait about 40 microseconds. Twiddle is available as a #define in wsd_settings.h
			__NOP_DELAY_LOOP(LMS_VCO_SETTLING_CYCLES);
			Lime_Read(LMS_PLL_VTUNE_R, &ulBuff);
			ulCH = ulBuff & LMS_PLL_VTUNE_H_M;
		}
		// clIndexCH should now point to the 2.5V transition point
		ulIndexCL = ulIndexCH;
		while ((ulCL == 0) && (ulIndexCL < 63))
		{
			// Increment the VCOCAP index and check new ulCL value
			// also, don't forget we have to preserve the upper bits of
			// LMS_PLL_VCOCAP_R
			Lime_FastWrite(LMS_PLL_VCOCAP_R, ulVCOCAPUpper | ++ulIndexCL);
			// wait about 40 microseconds. Twiddle is available as a #define in wsd_settings.h
			__NOP_DELAY_LOOP(LMS_VCO_SETTLING_CYCLES);
			Lime_Read(LMS_PLL_VTUNE_R, &ulBuff);
			ulCL = ulBuff & LMS_PLL_VTUNE_L_M;
		}
		// clIndexCL should now point to the 0.5V transition point
		// This CAN be the same point (e.g. 63), in which case this
		// can only be tuned to one value.
	}
	// CH = 0, CL = 1, we're to the right of the transition points
	// We will DEcrement the VCOCAP index until we find both transition points
	else if (ulCL)
	{
		PrintStr("CH = 0, CL = 1");
		__newline

		ulIndexCL = 31;
		while (ulCL && (ulIndexCL > 0))
		{
			// Decrement the VCOCAP index and check new ulCL value
			// also, don't forget we have to preserve the upper bits of
			// LMS_PLL_VCOCAP_R
			Lime_FastWrite(LMS_PLL_VCOCAP_R, ulVCOCAPUpper | --ulIndexCL);
			// wait about 40 microseconds. Twiddle is available as a #define in wsd_settings.h
			__NOP_DELAY_LOOP(LMS_VCO_SETTLING_CYCLES);
			Lime_Read(LMS_PLL_VTUNE_R, &ulBuff);
			ulCL = ulBuff & LMS_PLL_VTUNE_L_M;
		}
		// clIndexCL should now point to the 0.5V transition point
		ulIndexCH = ulIndexCL;
		while ((ulCH == 0) && (ulIndexCH > 0))
		{
			// Decrement the VCOCAP index and check new ulCH value
			// also, don't forget we have to preserve the upper bits of
			// LMS_PLL_VCOCAP_R
			Lime_FastWrite(LMS_PLL_VCOCAP_R, ulVCOCAPUpper | --ulIndexCH);
			// wait about 40 microseconds. Twiddle is available as a #define in wsd_settings.h
			__NOP_DELAY_LOOP(LMS_VCO_SETTLING_CYCLES);
			Lime_Read(LMS_PLL_VTUNE_R, &ulBuff);
			ulCH = ulBuff & LMS_PLL_VTUNE_H_M;
		}
		// clIndexCH should now point to the 2.5V transition point
		// This CAN be the same point (e.g. 0), in which case this
		// can only be tuned to one value.
	}
	// CH = 0, CL = 0, we're somewhere on the slope between VTUNE = hi and VTUNE = lo
	// We will search to the left and then to the right for the two transition points
	else
	{
		PrintStr("CH = 0, CL = 0");
		__newline

		ulIndexCL = 31;
		while ((ulCL == 0) && (ulIndexCL < 63))
		{
			// Increment the VCOCAP index and check new ulCL value
			// also, don't forget we have to preserve the upper bits of
			// LMS_PLL_VCOCAP_R
			Lime_FastWrite(LMS_PLL_VCOCAP_R, ulVCOCAPUpper | ++ulIndexCL);
			// wait about 40 microseconds. Twiddle is available as a #define in wsd_settings.h
			__NOP_DELAY_LOOP(LMS_VCO_SETTLING_CYCLES);
			Lime_Read(LMS_PLL_VTUNE_R, &ulBuff);
			ulCL = ulBuff & LMS_PLL_VTUNE_L_M;
		}
		// clIndexCL should now point to the 0.5V transition point
		// restart in middle to find the 2.5V transition point
		ulIndexCH = 31;
		while ((ulCH == 0) && (ulIndexCH > 0))
		{
			// Decrement the VCOCAP index and check new ulCH value
			// also, don't forget we have to preserve the upper bits of
			// LMS_PLL_VCOCAP_R
			Lime_FastWrite(LMS_PLL_VCOCAP_R, ulVCOCAPUpper | --ulIndexCH);
			// wait about 40 microseconds. Twiddle is available as a #define in wsd_settings.h
			__NOP_DELAY_LOOP(LMS_VCO_SETTLING_CYCLES);
			Lime_Read(LMS_PLL_VTUNE_R, &ulBuff);
			ulCH = ulBuff & LMS_PLL_VTUNE_H_M;
		}
		// clIndexCH should now point to the 2.5V transition point
		// These indices CANNOT be the same, since we couldn't have started in the
		// center and be at the edge of the Index range.
	}

	// The resulting calibrated value for VCOCAP is then:
	// VCOCAP = CH + (2.5 - VTune_Target)*(CL - CH)/(2.5 - 0.5)
	//        = CH + (CL - CH) / 2
	//        = (CH + CL)/2
	ulBuff = (ulIndexCH + ulIndexCL) >> 1;
	Lime_FastWrite(LMS_PLL_VCOCAP_R, ulVCOCAPUpper | ulBuff);

	PrintStr("CHInd: ");
	myLong2Hex(ulIndexCH, pcBuff, 2);
	PrintStr((unsigned char *)pcBuff);
	PrintStr(", CLInd: ");
	myLong2Hex(ulIndexCL, pcBuff, 2);
	PrintStr((unsigned char *)pcBuff);
	PrintStr(", VCOCAPInd: ");
	myLong2Hex(ulBuff, pcBuff, 2);
	PrintStr((unsigned char *)pcBuff);
	__newline

	if( (ulIndexCH == 0    && ulIndexCL == 0x3F)
	 || (ulIndexCH == 0    && ulIndexCL == 0   )
	 || (ulIndexCH == 0x3F && ulIndexCL == 0x3F)
	 || (ulIndexCH == 0x3F && ulIndexCL == 0   ))
	{
		Stel_ThrowError("VCO Comparator Invalid Val");
		errorsOccurred++;
	}

	// Restore the system clock settings. We're done setting stuff.
	errorsOccurred +=
					Lime_RobustWrite(LMS_CLK_CTRL_R, ulSavedClockCtrl);
//	errorsOccurred +=
//			Lime_SetClockCtrl(LMS_CLK_EN_RX_DSM_SPI | LMS_CLK_EN_TX_DSM_SPI, LMS_CLEAR);

	return errorsOccurred;
}

//*****************************************************************************
//
// Set the center frequency of the given PLL, and VCOCAP calibrate.
// This function uses some 64-bit math because of the size of the operands
// and the output precision required. TODO: verify settings.
//
// The valid frequency range is: [232500, 3720000] kHz
//
// Example: setting the TX center frequency to 205.5 MHz:
//          Lime_SetFrequency(LMS_TXPLL_BASE, 205500);
//
// Example: setting the RX center frequency to 2.45 GHz:
//			Lime_SetFrequency(LMS_RXPLL_BASE, 2450000);
//
// \param ulBaseAddr The base address of the TX or RX PLL.
// \param ulTargetFrequencyKHz The desired center frequency in kHz.
//
// \return A 0 upon success, or a 1 if an error occurs.
//
//*****************************************************************************
int
Lime_SetFrequency(const unsigned long ulBaseAddr, const unsigned long ulTargetFrequencyKHz)
{
	unsigned long ulNINT, ulNFRAC, ulFREQSEL, ulBuff;
	unsigned long long ullNFRAC;
//	unsigned long long ullBuff;
	char pcBuff[30];
	int errors = 0;

	// Make sure that the DSM_SPI clocks are set, otherwise any changes we make
	// to PLL configuration registers will never be committed from the shadow
	// shadow registers!
	errors +=
			Lime_SetClockCtrl(LMS_CLK_EN_RX_DSM_SPI | LMS_CLK_EN_TX_DSM_SPI, LMS_SET);

	//LMS_PLL_REF_CLK_RATE
	// NINT = 9 bits spread across LMS registers:
	//      LMS_PLL_NINT_OFFSET
	//		LMS_PLL_NFRAC_A_OFFSET
	// NFRAC = 23 bits spread across LMS registers:
	//		LMS_PLL_NFRAC_A_OFFSET
	//		LMS_PLL_NFRAC_B_OFFSET
	//		LMS_PLL_NFRAC_C_OFFSET

	// Set FREQSEL according to table 3.4.1, pg. 28 of:
	// LMS6002D Programming and Calibration Guide
	// We added two LSB zeros to the values in that table because the FREQSEL bits
	// are in [7-2] of the LMS_PLL_FREQSEL_OFFSET register, not [5-0]
	if       ((ulTargetFrequencyKHz >=  232500) && (ulTargetFrequencyKHz <=  285625)) {
		ulFREQSEL = 0x9C;	//0b10011100;
	} else if (ulTargetFrequencyKHz >  285625 && ulTargetFrequencyKHz <=  336875) {
		ulFREQSEL = 0xBC;	//0b10111100;
	} else if (ulTargetFrequencyKHz >  336875 && ulTargetFrequencyKHz <=  405000) {
		ulFREQSEL = 0xDC; 	//0b11011100;
	} else if (ulTargetFrequencyKHz >  405000 && ulTargetFrequencyKHz <=  465000) {
		ulFREQSEL = 0xFC;	//0b11111100;
	} else if (ulTargetFrequencyKHz >  465000 && ulTargetFrequencyKHz <=  571250) {
		ulFREQSEL = 0x98;	//0b10011000;
	} else if (ulTargetFrequencyKHz >  571250 && ulTargetFrequencyKHz <=  673750) {
		ulFREQSEL = 0xB8;	//0b10111000;
	} else if (ulTargetFrequencyKHz >  673750 && ulTargetFrequencyKHz <=  810000) {
		ulFREQSEL = 0xD8;	//0b11011000;
	} else if (ulTargetFrequencyKHz >  810000 && ulTargetFrequencyKHz <=  930000) {
		ulFREQSEL = 0xF8;	//0b11111000;
	} else if (ulTargetFrequencyKHz >  930000 && ulTargetFrequencyKHz <= 1142500) {
		ulFREQSEL = 0x94;	//0b10010100;
	} else if (ulTargetFrequencyKHz > 1142500 && ulTargetFrequencyKHz <= 1347500) {
		ulFREQSEL = 0xB4;	//0b10110100;
	} else if (ulTargetFrequencyKHz > 1347500 && ulTargetFrequencyKHz <= 1620000) {
		ulFREQSEL = 0xD4;	//0b11010100;
	} else if (ulTargetFrequencyKHz > 1620000 && ulTargetFrequencyKHz <= 1860000) {
		ulFREQSEL = 0xF4;	//0b11110100;
	} else if (ulTargetFrequencyKHz > 1860000 && ulTargetFrequencyKHz <= 2285000) {
		ulFREQSEL = 0x90;	//0b10010000;
	} else if (ulTargetFrequencyKHz > 2285000 && ulTargetFrequencyKHz <= 2695000) {
		ulFREQSEL = 0xB0;	//0b10110000;
	} else if (ulTargetFrequencyKHz > 2695000 && ulTargetFrequencyKHz <= 3240000) {
		ulFREQSEL = 0xD0;	//0b11010000;
	} else if (ulTargetFrequencyKHz > 3240000 && ulTargetFrequencyKHz <= 3720000) {
		ulFREQSEL = 0xF0;	//0b11110000;
	} else {
		Stel_ThrowError("Target Cen. Freq OOB of Radio");
		return 1;
	}

	// Write the selected value to the FREQSEL register.
	Lime_Read(ulBaseAddr + LMS_PLL_FREQSEL_OFFSET, &ulBuff);
	ulBuff &= ~LMS_PLL_FREQSEL_M;		// clear FREQSEL
	ulBuff |= ulFREQSEL;				// set new FREQSEL
	trx.ulRxFREQSEL = ulFREQSEL;
	if (Lime_RobustWrite(ulBaseAddr + LMS_PLL_FREQSEL_OFFSET, ulBuff))
	{
		errors = 1;
	}

	// Stellaris has built-in single-cycle multiply & hardware divider
	//
	// FREQSEL[2:0] is used to determine NINT and NFRAC
	// Note that FREQSEL[2:0] is LMS_PLL_FREQSEL_OFFSET bits [4:2]
	// because equation in LMS6002D programming guide
	// actually produces a decimal number if we're not careful,
	//
	// x = 2^(FREQSEL[2:0] - 3),
	// NINT = floor(x*f_lo/f_ref),
	// NFRAC = floor(2^23*{x*f_lo/f_ref - NINT}),
	//
	// we can use integer math if we reorder terms:
	//
	// NINT = (x*f_lo)/f_ref
	// NFRAC = (2^23*x*f_lo)/f_ref - 2^23*NINT
	//

	// If the Tx frequency is being changed, update Tx IQ and Tx LOFT calibration values
	// Tx LOFT is not frequency dependent, but it is band-dependent, and we're currently
	// planning on having band selection trigger all parameter selection, as represented
	// by the currently set center frequency for each Tx and Rx path, individually.
	if (ulBaseAddr == LMS_TXPLL_BASE)
	{
		// Load Tx IQ imbalance calibration settings for the new freq.
		// Also, load Tx LOFT calibration settings because with the new
		// frequency, we may be operating in a new frequency band!
		if (Lime_LoadFrequencyDependentCalValues(ulTargetFrequencyKHz, TX)
		 || Lime_LoadGainDependentCalValues(trx.ulCurrentTxGain) )
		{
			Stel_ThrowError("LoadTxCalValErr");
			errors++;
		}
	}
	// If the Rx frequency is being changed, update the Rx LOFT calibration values
	else if (ulBaseAddr == LMS_RXPLL_BASE)
	{
		// Load Tx IQ imbalance calibration settings for the new freq.
		// Also, load Tx LOFT calibration settings because with the new
		// frequency, we may be operating in a new frequency band!
		if (Lime_LoadFrequencyDependentCalValues(ulTargetFrequencyKHz, RX) )
		{
			Stel_ThrowError("LoadRxCalValErr");
			errors++;
		}
	}
	else
	{
		Stel_ThrowError("69");
		return 1;
	}

	// the first half of the calculation is just fine as 32-bit math
	ulFREQSEL = (ulFREQSEL & LMS_PLL_FREQSEL_NFRAC_M) >> 2;	// FREQSEL[2:0]
	ulFREQSEL = ulFREQSEL - 3;								// FREQSEL[2:0] - 3
	ulBuff = ulTargetFrequencyKHz << ulFREQSEL;				// f_lo*2^(FREQSEL[2:0]-3)
	ulNINT = ulBuff/LMS_PLL_REF_CLK_RATE;					// NOTE: single-cycle division

	// all this math needs to be performed with 64-bit precision due to the large
	// numbers involved. The Stellaris can handle this, no problem. Thanks, TI!
	ullNFRAC = (unsigned long long)8388608 * (unsigned long long)ulBuff;
	ullNFRAC = ullNFRAC/LMS_PLL_REF_CLK_RATE;
	ullNFRAC = ullNFRAC - (unsigned long long)8388608*(unsigned long long)ulNINT;
	ulNFRAC = (unsigned long)ullNFRAC;

	// debug output to make sure things are being done right
//	PrintStr("ulFREQSEL_M: ");
//	myLong2Hex(ulFREQSEL, pcBuff, 1);
//	PrintStr((unsigned char *)pcBuff);
//	__newline
//	PrintStr("Frequency: ");
//	myLong2Hex(ulTargetFrequencyKHz, pcBuff, 1);
//	PrintStr((unsigned char *)pcBuff);
//	PrintStr(", INT: ");
//	myLong2Hex(ulNINT, pcBuff, 1);
//	PrintStr((unsigned char *)pcBuff);
//	PrintStr(", FRAC: ");
//	myLong2Hex((unsigned long) ullNFRAC, pcBuff, 1);
//	PrintStr((unsigned char *)pcBuff);
//	__newline

	// Now, write the calculated values ulNINT and ulNFRAC to their
	// respective registers.
	// HI NINT
	ulBuff = (ulNINT & 0x01FE) >> 1;		// NINT[8:1]
	errors += Lime_RobustWrite(ulBaseAddr + LMS_PLL_NINT_OFFSET, ulBuff);

	// LO NINT, HI NFRAC
	ulBuff = (ulNINT & 0x0001) << 7;		// NINT[0]
	ulBuff |= (ulNFRAC & 0x7F0000) >> 16;	// NFRAC[22:16]
	errors += Lime_RobustWrite(ulBaseAddr + LMS_PLL_NFRAC_A_OFFSET, ulBuff);

	// MID NFRAC
	ulBuff = (ulNFRAC & 0x00FF00) >> 8;		// NFRAC[15:8]
	errors += Lime_RobustWrite(ulBaseAddr + LMS_PLL_NFRAC_B_OFFSET, ulBuff);

	//LO NFRAC
	ulBuff = (ulNFRAC & 0x0000FF);			// NFRAC[7:0]
	errors += Lime_RobustWrite(ulBaseAddr + LMS_PLL_NFRAC_C_OFFSET, ulBuff);

	// VCOCAP calibration is required whenever the channel is changed in order
	// to guarantee PLL lock and stability.
	errors += Lime_VCOCAPCalibration(ulBaseAddr);

	// Debug output for testing
	Lime_Read(ulBaseAddr + LMS_PLL_NINT_OFFSET, &ulBuff);
	myLong2Hex(ulBuff, pcBuff, 2);
	PrintStr("NIN: ");
	PrintStr((unsigned char *)pcBuff);
	Lime_Read(ulBaseAddr + LMS_PLL_NFRAC_A_OFFSET, &ulBuff);
	myLong2Hex(ulBuff, pcBuff, 2);
	PrintStr(", NFA: ");
	PrintStr((unsigned char *)pcBuff);
	Lime_Read(ulBaseAddr + LMS_PLL_NFRAC_B_OFFSET, &ulBuff);
	myLong2Hex(ulBuff, pcBuff, 2);
	PrintStr(", NFB: ");
	PrintStr((unsigned char *)pcBuff);
	Lime_Read(ulBaseAddr + LMS_PLL_NFRAC_B_OFFSET, &ulBuff);
	myLong2Hex(ulBuff, pcBuff, 2);
	PrintStr(", NFC: ");
	PrintStr((unsigned char *)pcBuff);
	__newline

	// Set Tx or Rx pathway settings for appropriate pathway.
	if (ulBaseAddr == LMS_TXPLL_BASE)
	{
		// Store the currently set center frequencies in global variables.
		// This is used for other software and for host reference.
		// More importantly, it's used by the functions we're about to call.
		trx.ulCurrentTxFreq = ulTargetFrequencyKHz;

		// Tx Frequency changed, so re-check for appropriate band settings.
		if (Lime_Tx_IsUHFBand())
		{
			// Set UHF transmit pathway
			errors += Lime_SelectTXVGA2(LMS_TXVGA2_PA1ON_PA2OFF);
			trx.isTransmittingInUHFBand = TRUE;
		}
		else if (Lime_Tx_IsWiFiBand())
		{
			// Set WiFi transmit pathway
			errors += Lime_SelectTXVGA2(LMS_TXVGA2_PA1OFF_PA2ON);
			trx.isTransmittingInUHFBand = FALSE;
		}
		else
		{
			Lime_SelectTXVGA2(LMS_TXVGA2_PA1OFF_PA2OFF);
			Stel_ThrowError("Unsupported Tx Center Frequency");
			errors++;
		}
	}
	else if (ulBaseAddr == LMS_RXPLL_BASE)
	{
		// Store the currently set center frequencies in global variables.
		// This is used for other software and for host reference.
		// More importantly, it's used by the functions we're about to call.
		trx.ulCurrentRxFreq = ulTargetFrequencyKHz;

		// Rx Frequency changed, so re-check for appropriate band settings.
		if (Lime_Rx_IsUHFBand())
		{
			errors += Lime_SelectActiveLNA(LMS_LNA_1);

			if (trx.TRXState == RX_NORMAL)
			{
				errors += Lime_SetRxInputPath(INTERNAL_LNA);
//				errors += Lime_DirectEnableInternalLNAs();
			}
			else if (trx.TRXState == RX_BYPASS_INT_LNA)
			{
				errors += Lime_SetRxInputPath(EXTERNAL_LNA);
//				errors += Lime_DirectDisableInternalLNAs();
			}
			trx.isReceivingInUHFBand = TRUE;
		}
		else if (Lime_Rx_IsWiFiBand())
		{
			// Set WiFi receive pathway
			if (trx.TRXState == RX_BYPASS_INT_LNA)
			{
				PrintStr("WiFi Chain Does Not Support Internal LNA Bypass Mode");
				__newline
			}
//			errors += Lime_DirectEnableInternalLNAs();
			errors += Lime_SelectActiveLNA(LMS_LNA_2);
			errors += Lime_SetRxInputPath(INTERNAL_LNA);
			trx.isReceivingInUHFBand = FALSE;
		}
		else
		{
			// Default to LNA3 broadband input
			Lime_SelectActiveLNA(LMS_LNA_3);
			Stel_ThrowError("Unsupported Rx Center Frequency");
			errors++;
		}
	}
	else
	{
		Stel_ThrowError("58");
		errors++;
	}

	// REG: redundant code is redundant
//	// Store the currently set center frequencies in global variables.
//	// This is used for other software and for host reference.
//	if (ulBaseAddr == LMS_TXPLL_BASE)
//	{
////		g_ulCurrentTxFreq = ulTargetFrequencyKHz;
//		trx.ulCurrentTxFreq = ulTargetFrequencyKHz;
//	}
//	else if (ulBaseAddr == LMS_RXPLL_BASE)
//	{
//		trx.ulCurrentRxFreq = ulTargetFrequencyKHz;
//	}
//	else
//	{
//		Stel_ThrowError("58");
//		errors++;
//	}

	return errors;
}

//*****************************************************************************
//
// Sets the LPF bandwidth of the given TX/RX base LPF register. (0x34, 0x54)
//
// The following are valid filter single-sideed bandwidths:
// 0.75, 0.875, 1.25, 1.375, 1.5, 1.92, 2.5, 2.75, 3.0, 3.5
// 4.375, 5.0, 6.0, 7.0, 10.0, 14.0 MHz
//
// Constants to pass as argument to this function exist in the format:
//  LMS_LPF_BW_X_Y where X is the integer value and Y is the decimal value.
//
// \param ulBaseAddr The base address of the RX/TX LPF: LMS_TXLPF_BASE, LMS_RXLPF_BASE
// \param ulBandwidthSetting The bandwidth setting for the given LPF. See desc. for more info.
//
// \return A 0 upon success, or a 1 on failure.
//
//*****************************************************************************
int
Lime_SetFilterBandwidth(const unsigned long ulBaseAddr, const unsigned long ulBandwidthSetting)
{
	unsigned long errors = 0;

	//TODO - CFB_TIA_RXFE cap can be set for RXVGA1 to add additional filtering: 0x77
	//		 This is BW dependant, so should be set inside this function so both BW
	//		 setting occur at the same time.

	switch(ulBaseAddr)
	{
		case LMS_TXLPF_BASE:
		{
			PrintStr("TX BW = ");
			break;
		}
		case LMS_RXLPF_BASE:
		{
			PrintStr("RX BW = ");
			break;
		}
		default:
		{
			//bad value
			Stel_ThrowError("19");
			return 1;
		}
	}

	// Display the set SSB channel BW.
	switch(ulBandwidthSetting)
	{
		case LMS_LPF_BW_14:
			{
				PrintStr("28M");
				break;
			}
		case LMS_LPF_BW_10:
			{
				PrintStr("20M");
				break;
			}
		case LMS_LPF_BW_7:
			{
				PrintStr("14M");
				break;
			}
		case LMS_LPF_BW_6:
			{
				PrintStr("12M");
				break;
			}
		case LMS_LPF_BW_5:
			{
				PrintStr("10M");
				break;
			}
		case LMS_LPF_BW_4_375:
			{
				PrintStr("8.75M");
				break;
			}
		case LMS_LPF_BW_3_5:
			{
				PrintStr("7M");
				break;
			}
		case LMS_LPF_BW_3:
			{
				PrintStr("6M");
				break;
			}
		case LMS_LPF_BW_2_75:
			{
				PrintStr("5.5M");
				break;
			}
		case LMS_LPF_BW_2_5:
			{
				PrintStr("5M");
				break;
			}
		case LMS_LPF_BW_1_92:
			{
				PrintStr("3.84M");
				break;
			}
		case LMS_LPF_BW_1_5:
			{
				PrintStr("3M");
				break;
			}
		case LMS_LPF_BW_1_375:
			{
				PrintStr("2.75M");
				break;
			}
		case LMS_LPF_BW_1_25:
			{
				PrintStr("2.5M");
				break;
			}
		case LMS_LPF_BW_0_875:
			{
				PrintStr("1.75M");
				break;
			}
		case LMS_LPF_BW_0_75:
			{
				PrintStr("1.5M");
				break;
			}
		default:
			{
				Stel_ThrowError("Invalid BW");
			}
	}
	__newline

	// Note that BWC is in field [5-2], not [3-0]
	errors += Lime_SetRegisterField(ulBaseAddr + LMS_LPF_BWC_OFFSET,
								    ulBandwidthSetting << 2,
							        LMS_LPF_BWC_M);
	// Keep the Rx LPF setting and the CalLPF setting in lockstep.
	// BWC in 0x07 IS in [3-0]
	if (ulBaseAddr == LMS_RXLPF_BASE)
	{
		errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R,
									    ulBandwidthSetting,
									    LMS_LPFCAL_2_BWC_M);
	}
	return errors;
}

//*****************************************************************************
//
// Set the current operational state of the Top Module EN, STXEN, SRXEN, SRESET.
// Default should be: LMS_TOP_SRESET_PD | LMS_TOP_EN | LMS_TOP_STXEN | LMS_TOP_SRXEN
//
// This function ALWAYS enables DECODE mode, as well as 4-wire SPI
//
// \param ulBitMask The bit-packed setting for the TOP_SOFTEN_R, 0x05.
//
//*****************************************************************************
int
Lime_SetSoftEnable(unsigned long ulBitMask)
{
	// Always remain in DECODE mode (rather than DIRECT CONTROL)
	// Always enable SPI 4-wire mode
	ulBitMask &= ~LMS_TOP_CONTROL_M;
	ulBitMask |= LMS_TOP_TFWMODE_M;

	return Lime_RobustWrite(LMS_TOP_SOFTEN_R, ulBitMask);
}

//*****************************************************************************
//
// Clear the Soft Tx Enable register for the LMS6002D.
//
// \return A zero if successful. A 1 if error occurred.
//
//*****************************************************************************
int
Lime_SoftTXDisable(void)
{
	return Lime_SetRegisterField(LMS_TOP_SOFTEN_R, 0x00, LMS_TOP_STXEN);
}

//*****************************************************************************
//
// Set the Soft Tx Enable register for the LMS6002D.
//
// \return A zero if successful. A 1 if error occurred.
//
//*****************************************************************************
int
Lime_SoftTXEnable(void)
{
	return Lime_SetRegisterField(LMS_TOP_SOFTEN_R, 0xFF, LMS_TOP_STXEN);
}

//*****************************************************************************
//
// Clear the Soft Rx Enable register for the LMS6002D.
//
// \return A zero if successful. A 1 if error occurred.
//
//*****************************************************************************
int
Lime_SoftRXDisable(void)
{
	return Lime_SetRegisterField(LMS_TOP_SOFTEN_R, 0x00, LMS_TOP_SRXEN);
}

//*****************************************************************************
//
// Set the Soft Rx Enable register for the LMS6002D.
//
// \return A zero if successful. A 1 if error occurred.
//
//*****************************************************************************
int
Lime_SoftRXEnable(void)
{
	return Lime_SetRegisterField(LMS_TOP_SOFTEN_R, 0xFF, LMS_TOP_SRXEN);
}

//*****************************************************************************
//
// Toggle the Soft TXEN register for the LMS6002D.
//
// \return A zero if successful. A 1 if error occurred.
//
//*****************************************************************************
int
Lime_ToggleSTXEN(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_TOP_SOFTEN_R, &ulBuff);
	ulBuff ^= LMS_TOP_STXEN;

	if (ulBuff & LMS_TOP_STXEN)
		{
			PrintStr("Soft TX Enabled");
		}
		else
		{
			PrintStr("Soft TX DISabled");
		}
		__newline

	return Lime_SetSoftEnable(ulBuff);
}

//*****************************************************************************
//
// Toggle the Soft RXEN register for the LMS6002D.
//
// \return A zero if successful. A 1 if error occurred.
//
//*****************************************************************************
int
Lime_ToggleSRXEN(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_TOP_SOFTEN_R, &ulBuff);
	ulBuff ^= LMS_TOP_SRXEN;

	if (ulBuff & LMS_TOP_SRXEN)
	{
		PrintStr("Soft RX Enabled");
	}
	else
	{
		PrintStr("Soft RX DISabled");
	}
	__newline

	return Lime_SetSoftEnable(ulBuff);
}

//*****************************************************************************
//
// Set the clock distribution control register for various modules.
// The following are acceptable clocks to set/clear:
//
//	LMS_CLK_EN_RXOUTSW
//	LMS_CLK_EN_PLLCLKOUT
//	LMS_CLK_EN_LPFCAL
//	LMS_CLK_EN_RX_VGA2_DCCAL
//	LMS_CLK_EN_RX_LPF_DCCAL
//	LMS_CLK_EN_RX_DSM_SPI
//	LMS_CLK_EN_TX_LPF_DCCA
//	LMS_CLK_EN_TX_DSM_SPI
//
// \param ulClkMask Bitmask of the clock(s) to set/clear.
// \param ulOperation either LMS_SET or LMS_CLEAR to indicate the desired operation.
//
// \return A 0 if successful or a 1 if an error occurred.
//
//*****************************************************************************
int
Lime_SetClockCtrl(const unsigned long ulClkMask, const unsigned long ulOperation)
{
	unsigned long ulBuff;
	int iErrorOccurred = 0;
//	char pcBuff[4];

	if (ulOperation == LMS_SET)
	{
		Lime_Read(LMS_CLK_CTRL_R, &ulBuff);
		ulBuff |= ulClkMask;	// set desired CLKEN bits
		iErrorOccurred +=
				Lime_RobustWrite(LMS_CLK_CTRL_R, ulBuff);
	}
	else
	{
		Lime_Read(LMS_CLK_CTRL_R, &ulBuff);
		ulBuff &= ~ulClkMask;	// clear desired CLKEN bits
		iErrorOccurred +=
				Lime_RobustWrite(LMS_CLK_CTRL_R, ulBuff);
	}

	// Debug output.
//	PrintStr("CLK CTRL: 0x");
//	myLong2Hex(ulBuff, pcBuff, 2);
//	PrintStr((unsigned char *)pcBuff);
//	__newline

	return iErrorOccurred;
}

//*****************************************************************************
//
// DEPRECATED
// Log-Linear gain setting for TX VGA1. Takes values: [0, 31] <=> [-35, -4] dB.
// Should be linear in 1 dB increments. We need to test this.
// The raw gain registers are located in 0x4B, but we should leave those alone.
//
// \param ulVal A gain setting between 0 and 31.
//
// \return A 0 if successful, or 1 on error.
//
//*****************************************************************************
int
Lime_SetTXVGA1Gain(const unsigned long ulVal)
{
	char pcBuff[5];

	if (ulVal > 31) //can't be less than 0
	{
		//illegal value
		return 1;
	}

	// debug output
	myLong2Hex(ulVal, pcBuff, 2);
	PrintStr("Set TX_VGA1: ");
	PrintStr((unsigned char *)pcBuff);
	__newline;

	return Lime_SetRegisterField(LMS_GAIN_TXVGA1_R, ulVal, LMS_GAIN_TXVGA1_M);
}

//*****************************************************************************
//
// DEPRECATED
// Log-Linear gain setting for TX VGA2. Takes values: [0, 25] <=> [0, 25] dB.
// Should be linear in 1 dB increments. We need to test this.
//
// \param ulVal A gain setting between 0 and 25.
//
// \return A 0 if successful, or 1 on error.
//
//*****************************************************************************
int
Lime_SetTXVGA2Gain(const unsigned long ulVal)
{
	char pcBuff[5];

	if (ulVal > 25) //can't be less than 0
	{
		// illegal value!
		return 1;
	}

	// debug output
	myLong2Hex(ulVal, pcBuff, 2);
	PrintStr("Set TX_VGA2: ");
	PrintStr((unsigned char *)pcBuff);
	__newline;

	return Lime_SetRegisterField(LMS_GAIN_TXVGA2_R, ulVal << 3, LMS_GAIN_TXVGA2_M);
}

//*****************************************************************************
//
// Log-Linear master Tx gain setting for the LMS6002D in [0, 56] This should
// always be used since it ensures that the Tx gain settings remain in synch
// with the Tx calibration values and global variables.
//
// \param ulNewGain the new gain setting <0, 56>
//
// \return a zero on success, or non-zero on error.
//
//*****************************************************************************
int
Lime_SetMasterTxGain(const unsigned long ulNewGain)
{
	int errors = 0;

	if (ulNewGain > 56)
	{
		return 1;
	}

	// Calibration showed that TxVGA gain was more consistent when G (TxVGA1)
	// was maximized first, before H (TxVGA2) was increased. This only held
	// true for G up to 25. After that, it was just better to keep both values
	// as low as possible while still meeting end-to-end gain requirement.
	// You'll find that for gain > 45 or so, the output is kinda poopy.
	if (ulNewGain <= 25)
	{
		errors += Lime_SetTXVGA1Gain(ulNewGain);
		errors += Lime_SetTXVGA2Gain(0);
	}
	else if (ulNewGain <= 50)
	{
		errors += Lime_SetTXVGA1Gain(25);
		errors += Lime_SetTXVGA2Gain(ulNewGain - 25);
	}
	else //ulNewGain = [51, 56]
	{
		errors += Lime_SetTXVGA1Gain(ulNewGain - 25);
		errors += Lime_SetTXVGA2Gain(25);
	}

	// Load the stored Tx LOFT calibration values for this gain setting.
	Lime_LoadGainDependentCalValues(ulNewGain);
	// Store global variable for use in code and host application
	trx.ulCurrentTxGain = ulNewGain;
	return errors;
}

//*****************************************************************************
//
// Log-Linear gain setting for RX VGA2. Takes values: [0, 10] <=> [0, 30] dB.
// Should be linear in 3 dB increments. We need to test this.
// Technically, the VGA can go up to 60 dB, but it's not recommended.
//
// \param ulVal A gain setting between 0 and 10.
//
// \return A 0 if successful, or 1 on error.
//
//*****************************************************************************
int
Lime_SetRXVGA2Gain(const unsigned long ulVal)
{
	char pcBuff[5];

	if (ulVal > 10) //can't be less than 0
	{
		// illegal value!
		return 1;
	}

	//debug output
	myLong2Hex(ulVal*3, pcBuff, 2);
	PrintStr("Set RX_VGA2: ");
	PrintStr((unsigned char *)pcBuff);
	__newline;

	return Lime_SetRegisterField(LMS_GAIN_RXVGA2_R, ulVal, LMS_GAIN_RXVGA2_M);
}

//*****************************************************************************
//
// NON-Linear gain setting for RX VGA1. Takes values: [0, 127] <=> [~0, 30+] dB.
// This gain is explicitly not linear, so we need to measure how it works.
//
// Note that this register is actually labeled "RFB_TIA_RXFE" in the RXFE module.
//
// \param ulVal A gain setting between 0 and 127.
//
// \return A 0 if successful, or 1 on error.
//
//*****************************************************************************
int
Lime_SetRXVGA1Gain(const unsigned long ulVal)
{
	char pcBuff[5];

	if (ulVal > 120) //can't be less than 0
	{
		// illegal value!
		return 1;
	}

	// debug output
	myLong2Hex(ulVal, pcBuff, 2);
	PrintStr("Set RX_VGA1: ");
	PrintStr((unsigned char *)pcBuff);
	__newline;

	return Lime_SetRegisterField(LMS_GAIN_RXVGA1_R, ulVal, LMS_GAIN_RXVGA1_M);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
int
Lime_SelectTXVGA2_PA(const unsigned long ulMask)
{
	//TODO - select between TXVGA2 PA1, PA2, enable loopback PA, or PKDET PA: 0x44
	return 1;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
//int
//Lime_LPFBypassSet(const unsigned long ulBaseAddr, const unsigned long ulMask)
//{
//	//TODO - enable/disable the LPF bypass switch. 0x35 0x55 BIT6
//	// default 0: normal operation
//	return 1;
//}

//*****************************************************************************
//
// Select which LNA is active: 1, 2, 3, or none. Keep LNAs and PLL buffers synchronized.
// Keeps LNASEL (0x75) and SELOUT (in 0x25) synchronized.
//
// Accepted passed arguments are:
//
//	LMS_LNA_DISABLED
//	LMS_LNA_1
//	LMS_LNA_2
//	LMS_LNA_3
//
// \param ulSetting Which LNA (or none) should be active? See desc. for valid values.
//
// \return A 0 if successful, or a 1 on error.
//
//*****************************************************************************
int
Lime_SelectActiveLNA(unsigned long ulSetting)
{
//	unsigned long ulSavedClockCtrl;
	int iErrorOccurred = 0;
//	char cBuff[5];

	// debug - display which LNA is being set
//	myLong2Hex(ulSetting >> 4, cBuff, 1);
//	PrintStr("LNA Selection: ");
//	PrintStr((unsigned char *)cBuff);
//	__newline

	// Set which LNA should be powered (or none)
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_LNA_CTRL_R, ulSetting, LMS_LNA_CTRL_LNASEL_M);

	// Set which RX PLL buffer should be powered (or none)
	// Note that the field in LMS_LNA_CTRL_R is [5:4],
	// whereas in LMS_RXPLL_SELOUT_R it is [1:0],
	// but at least the settings mirror each other.
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_RXPLL_BASE + LMS_PLL_FREQSEL_OFFSET,
					              (ulSetting >> 4),
					              LMS_PLL_SELOUT_M);

	return iErrorOccurred;
}

//*****************************************************************************
//
// Select which RF Loopback path is active: 1, 2, 3, or none. Keeps LB and PLL buffers synchronized.
// This synchronization is accomplished through synchronizing LBRFEN (in 0x08)
// and SELOUT (in 0x25).
// See Fig 3.2 in LMS6002D "Programming & Calibration Guide"
//
// This function also sets g_currentOperationalMode when RF Loopback is NOT
// being disabled.
//
// Accepted passed arguments are:
//
//	LMS_RF_LOOPB_DISABLED
//	LMS_RF_LOOPB_LNA_1
//	LMS_RF_LOOPB_LNA_2
//	LMS_RF_LOOPB_LNA_3
//
// This function also disables any BB loopback setting unless RF loopback is disabled.
//
// \param ulSetting Argument specifying the RF Loopback setting. See comments for details.
//
// \return A zero if no error occurred. Otherwise, the positive number of errors.
//
//*****************************************************************************
int
Lime_SelectRFLoopback(unsigned long ulSetting)
{
	int errorsOccurred = 0;
	char cBuff[5];

	// debug - display which RF Loopback path is set
	myLong2Hex(ulSetting, cBuff, 1);
	PrintStr("RF Loopback Selection: ");
	PrintStr((unsigned char *)cBuff);
	__newline

	// Make sure BB Loop-back is DISabled IFF any RF loopback path is selected
	if (ulSetting != LMS_RF_LOOPB_DISABLED)
	{
		// Set global mode state
		trx.CalibrationMode = LMS_CAL_RF_LOOPBACK;

		// Disable BB loopback, explicitly
		errorsOccurred +=
				Lime_SelectBBLoopback(LMS_BB_LOOPB_DISABLED);

		// Enable RF loopback switch in Top module - not sure if this is necessary
		errorsOccurred += Lime_SetRegisterField(LMS_TOP_MISC_CONFIG_R,
										  	    LMS_TOP_RFLOOB_SW_ON_M,
										  	    LMS_TOP_RFLOOB_SW_ON_M);
	}
	else
	{
		// Disable RF loopback switch in Top module
		errorsOccurred += Lime_SetRegisterField(LMS_TOP_MISC_CONFIG_R,
												0x00,
												LMS_TOP_RFLOOB_SW_ON_M);
	}

	// Set which loopback path is enabled.
	errorsOccurred +=
			Lime_SetRegisterField(LMS_LOOPBACK_CTRL_R, ulSetting, LMS_LOOPBACK_LBRFEN_M);

	// Set which RX PLL buffer should be powered (or none)
	// I've ensured all calls to this function have the correct setting. REG 12/10/2013
	errorsOccurred +=
			Lime_SetRegisterField(LMS_RXPLL_BASE + LMS_PLL_FREQSEL_OFFSET,
					              ulSetting,
					              LMS_PLL_SELOUT_M);

	return errorsOccurred;
}

//*****************************************************************************
//
// Select which BB loopback mode is enabled. Ensures safe settings.
// See Fig 3.2 in LMS6002D "Programming & Calibration Guide"
//
// This function also sets g_currentOperationalMode when BB Loopback is NOT
// being disabled.
//
// Accepted passed arguments are:
//
//	LMS_BB_LOOPB_DISABLED
//	LMS_BB_LOOPB_LPFIN
//	LMS_BB_LOOPB_VGA2IN
//	LMS_BB_LOOPB_OPIN
//
// This function also disables any RF loopback setting unless BB loopback is disabled.
//
// \param Argument specifying BB Loopback setting. See function comments for details.
//
// \return A zero if successful, and the number of errors if not.
//
//*****************************************************************************
int
Lime_SelectBBLoopback(unsigned long ulSetting)
{
	int errors = 0;
	char cBuff[5];

	// debug - display which RF Loopback path is set
	myLong2Hex(ulSetting >> 4, cBuff, 1);
	PrintStr("BB Loopback Selection: ");
	PrintStr((unsigned char *)cBuff);
	__newline

	// Make sure RF Loopback is DISabled if any BB Loopback path is selected
	if (ulSetting != LMS_BB_LOOPB_DISABLED)
	{
		// Set global signal mode state.
		trx.CalibrationMode = LMS_CAL_BB_LOOPBACK;

		// Disable RF Loopback explicitly.
		errors +=
				Lime_SelectRFLoopback(LMS_RF_LOOPB_DISABLED);
	}

	// Some settings have different requirements like powering down RXVGA1,
	// RXVGA2 or RXLPF
	switch(ulSetting)
	{
		case LMS_BB_LOOPB_OPIN:
		{
			// Power down RxVGA2...
			// REG: Based on LMS6002D protocol analyzer, I'm attempting to also
			// set the common-mode voltage level to 0.82 V, but not sure if that matters.
			errors += Lime_SetRegisterField(LMS_RXVGA2_CTRL_R,
									   0x34,
									   0xFF | LMS_RXVGA2_CTRL_EN_M);
			//fall through
		}
		case LMS_BB_LOOPB_VGA2IN:
		{
			// Power down the Rx LPF Block...
			errors += Lime_SetRegisterField(LMS_RXLPF_BASE + LMS_LPF_BWC_OFFSET,
											0x00,
											LMS_LPF_LPFEN_M);
			//fall through
		}
		case LMS_BB_LOOPB_LPFIN:
		{
			// Power down RXVGA1 (RXTIA in Fig 3.1 in most recent manual).
			// REG: also actually powered down EVERYTHING as per protocol analyzer
			errors +=
					Lime_SetRegisterField(LMS_LNA_DIRECT_R,
										  0x00,
										  0xFF | LMS_LNA_DIRECT_PD_TIA_RXFE_M);
			break;
		}
		default:
		{
			// do nothing
		}
	}

	// Set which loopback path is enabled. While the bits are listed individually,
	// I've defined these bit masks so that you set all three simultaneously. That
	// way you can't have any two BB loopback targets active at the same time. It's
	// not clear this is correct, but it doesn't really make sense otherwise.
	errors +=
			Lime_SetRegisterField(LMS_LOOPBACK_CTRL_R, ulSetting, LMS_LOOPBACK_LBBBEN_M);

	return errors;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
int
Lime_SetDACSettings(unsigned long ulMask)
{
	//TODO - Enable/Disable the ADC/DACs in software in order to calibration w/o
	// 		 TX input: 0x57
	//TODO - DAC internal output load resistor ctrl: 0x57, TX_CTRL
	//TODO - DAC full scale output curent control: 0x57, TX_CTRL
	//\param Bit mask for various TX_CTRL settings for the DAC.
	return 1;
}

//*****************************************************************************
//
//
//
//*****************************************************************************
int
Lime_SetADCSettings(unsigned long ulMask)
{
	//TODO - Enable/Disable the ADC/DACs in software in order to calibration w/o
	// 		 TX input: 0x57
	//TODO - DAC internal output load resistor ctrl: 0x57, TX_CTRL
	//TODO - DAC full scale output curent control: 0x57, TX_CTRL
	return 1;
}

//*****************************************************************************
//
// Enable or disable the DAC/ADC modules. This has recently been modified in
// response to this Myriad RF forum thread:
// https://groups.google.com/forum/#!topic/limemicro-opensource/PojHSCh8lso
//
// "DAC's introduce some DC component to TX path while it is power on. Best
// DC offset calibration result is achieved when DAC are power on and are
// set to 0+0*i."
//
// M@#%($&FU@#%%$#ERS. SERIOUSLY, LIME?! Could you not have fucking written
// that into your goddamned calibration module? Why do I have to search
// online community forums full of garbage to find this offhand comment?
// UGH.
//
// \param LMS_SET or LMS_CLEAR to enable or disable the ADC/DAC modules.
//
// \return a zero upon success
//
//*****************************************************************************
int
Lime_SetDACEnable(En_Dis_t ulState)
{
//	unsigned long ulBuff;
	int errors = 0;

	if (ulState == ENABLE)
	{
		if (trx.verboseOutputEnabled)
		{
			PrintStr("DAC input restored");
			__newline
		}

		// Tell the BB to restore the normal DAC input.
		errors += BB_SetDACInputState(NORMAL);

//		Lime_Read(LMS_DAC_CTRL_R, &ulBuff);
//		ulBuff |= LMS_DACADC_EN_M; 	// set EN_ADC_DAC
//		return Lime_RobustWrite(LMS_DAC_CTRL_R, ulBuff);
	}
	else //DISABLE
	{
		if (trx.verboseOutputEnabled)
		{
			PrintStr("Zeroing DAC input");
			__newline
		}

		// Don't disable the DAC, rather input all zero samples to it.
		errors += BB_SetDACInputState(ALLZERO);

//		Lime_Read(LMS_DAC_CTRL_R, &ulBuff);
//		ulBuff &= ~LMS_DACADC_EN_M; 	// clear EN_ADC_DAC
//		return Lime_RobustWrite(LMS_DAC_CTRL_R, ulBuff);
	}

	return errors;
}

//*****************************************************************************
//
// Default misc. radio settings. This should be set on startup.
//
//*****************************************************************************
int
Lime_SetDefaultSettings(void)
{
	int iErrorOccurred = 0;

	// Recommended PLL settings: CP Current = 1200 uA
	// CP Up Offset = 30 uA, CP Down Offset = 0 uA
	// These are apparently chosen to optimize the default PLL loop filter design
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_TXPLL_BASE + LMS_PLL_ICHP_OFFSET,
					LMS_PLL_ICHP_1200_uA, LMS_PLL_ICHP_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_TXPLL_BASE + LMS_PLL_OFFUP_OFFSET,
					LMS_PLL_OFFUP_30_uA, LMS_PLL_OFFUP_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_TXPLL_BASE + LMS_PLL_OFFDOWN_OFFSET,
					LMS_PLL_OFFDOWN_0_uA, LMS_PLL_OFFDOWN_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_RXPLL_BASE + LMS_PLL_ICHP_OFFSET,
					LMS_PLL_ICHP_1200_uA, LMS_PLL_ICHP_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_RXPLL_BASE + LMS_PLL_OFFUP_OFFSET,
					LMS_PLL_OFFUP_30_uA, LMS_PLL_OFFUP_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_RXPLL_BASE + LMS_PLL_OFFDOWN_OFFSET,
					LMS_PLL_OFFDOWN_0_uA, LMS_PLL_OFFDOWN_M);

	//TODO - Also, explicitly DC-couple the PLLCLK.

	// Max all the TX bias currents to increase linearity
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_TXRF_TXLOBUF_R, LMS_TXRF_LOBUF_GOODCURRENT, LMS_TXRF_ICT_TXLOBUF_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_TXRF_TXMIX_R, LMS_TXRF_ICT_TXMIX_21mA, LMS_TXRF_ICT_TXMIX_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_TXRF_TXDRV_R, LMS_TXRF_ICT_TXDRV_22mA, LMS_TXRF_ICT_TXDRV_M);

	//TODO - play with the VBCAS setting for linearity: 0x47

	// set RXVGA2 common mode voltage: recommended is 12 (780mV): 0x64 VCM
	// Lime FAQ rev 11, #5.27
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_RXVGA2_CTRL_R,
								  RXVGA2_CTRL_CMV_DEFAULT,
								  LMS_RXVGA2_CTRL_CMVC_M);

	//TODO - we can play with CBE_LNA_RXFE cap values for matching: 0x75

	// Lime Microsystems recommends that for the default PLL Loop filter, we
	// should set the ADC settings:
	// Reference Bias Resistor to 10 uA,
	// Reference Gain Adjust to 1.75 V,
	// Common Mode Adjust to 960 mV.
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_ADC_CTRL_1_R, LMS_ADC_REFBIAS_10uA, LMS_ADC_REF_BIAS_RES_ADJ_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_ADC_CTRL_2_R, LMS_ADC_REFGAIN_1_75V, LMS_ADC_REF_GAIN_ADJ_M);
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_ADC_CTRL_2_R, LMS_ADC_COMMODE_960mV, LMS_ADC_COMM_MODE_ADJ_M);

	// Lime Microsystems recommends that the LNA load resistor be changed from default in
	// order to improve LNA gain.
	iErrorOccurred +=
			Lime_SetRegisterField(LMS_LNA_RDLINT_R, LMS_LNA_OPTIMAL_BIASRES, LMS_LNA_RDLINT_M);

	PrintStr("Default PLL, TXRF, and ADC settings were set.");
	__newline

	return iErrorOccurred;
}

//*****************************************************************************
//
// Writes a given value to the masked field in a given LMS6002D config register.
// Preserves the values not specified in ulFieldMask.
//
// Checks if PLL configuration registers are being written, and makes sure the
// PLL DSM SPI clocks are enabled.
//
// \param ulRegister The register to write values to.
// \param ulVal The value to write to that register. This can be any long value,
//              but only the bits specified by ulFieldMask will be written.
// \param ulFieldMask The bit-packed representation of the specific bits to write
//                    from ulVal to ulRegister. All non-1 bits will be untouched.
//
// \return A 0 upon success, or a 1 if an error occurs.
//
//*****************************************************************************
int
Lime_SetRegisterField(const unsigned long ulRegister, unsigned long ulVal, const unsigned long ulFieldMask)
{
	unsigned long ulBuff;
	unsigned long ulSavedClockCtrl;
	unsigned int errors = 0;
	unsigned int clkResetNeeded = 0;

	// If the LMS register block is LMS_TXPLL_BASE or LMS_RXPLL_BASE, then
	// the PLL SPI DSM clock must be enabled in order to force the PLL shadow
	// registers to write the real configuration registers.
	ulBuff = (ulRegister & LMS_BASE_MASK);
	if (ulBuff == LMS_TXPLL_BASE)
	{
		Lime_Read(LMS_CLK_CTRL_R , &ulSavedClockCtrl);
		Lime_SetClockCtrl(LMS_CLK_EN_TX_DSM_SPI, LMS_SET);
		clkResetNeeded = 1;
	}
	else if (ulBuff == LMS_RXPLL_BASE)
	{
		Lime_Read(LMS_CLK_CTRL_R , &ulSavedClockCtrl);
		Lime_SetClockCtrl(LMS_CLK_EN_RX_DSM_SPI, LMS_SET);
		clkResetNeeded = 1;
	}

	// Write to the targeted register, taking care not to disturb any masked
	// bits in the register.
	Lime_Read(ulRegister, &ulBuff);
	ulBuff &= ~ulFieldMask;				//clear the field
	ulBuff |= (ulVal & ulFieldMask);	//set the field
	errors += Lime_RobustWrite(ulRegister, ulBuff);

	// restore the Clock Control Register, if it was modified.
	if (clkResetNeeded)
	{
		errors += Lime_RobustWrite(LMS_CLK_CTRL_R, ulSavedClockCtrl);
	}

	return errors;
}

//*****************************************************************************
//
// Writes a given value to the masked field in a given LMS6002D config register.
// Preserves the values not specified in ulFieldMask.
//
// Checks if PLL configuration registers are being written, and makes sure the
// PLL DSM SPI clocks are enabled.
//
// \param ulRegister The register to write values to.
// \param ulVal The value to write to that register. This can be any long value,
//              but only the bits specified by ulFieldMask will be written.
// \param ulFieldMask The bit-packed representation of the specific bits to write
//                    from ulVal to ulRegister. All non-1 bits will be untouched.
//
// \return A 0 upon success, or a 1 if an error occurs.
//
//*****************************************************************************
void
Lime_SetRegisterField_Fast(const unsigned long ulRegister, unsigned long ulVal, const unsigned long ulFieldMask)
{
	unsigned long ulBuff;
	unsigned long ulSavedClockCtrl;
	unsigned int clkResetNeeded = 0;

	// If the LMS register block is LMS_TXPLL_BASE or LMS_RXPLL_BASE, then
	// the PLL SPI DSM clock must be enabled in order to force the PLL shadow
	// registers to write the real configuration registers.
	ulBuff = (ulRegister & LMS_BASE_MASK);
	if (ulBuff == LMS_TXPLL_BASE)
	{
		Lime_Read(LMS_CLK_CTRL_R , &ulSavedClockCtrl);
		Lime_SetClockCtrl(LMS_CLK_EN_TX_DSM_SPI, LMS_SET);
		clkResetNeeded = 1;
	}
	else if (ulBuff == LMS_RXPLL_BASE)
	{
		Lime_Read(LMS_CLK_CTRL_R , &ulSavedClockCtrl);
		Lime_SetClockCtrl(LMS_CLK_EN_RX_DSM_SPI, LMS_SET);
		clkResetNeeded = 1;
	}

	// Write to the targeted register, taking care not to disturb any masked
	// bits in the register.
	Lime_Read(ulRegister, &ulBuff);
	ulBuff &= ~ulFieldMask;				//clear the field
	ulBuff |= (ulVal & ulFieldMask);	//set the field
	Lime_FastWrite(ulRegister, ulBuff);

	// restore the Clock Control Register, if it was modified.
	if (clkResetNeeded)
	{
		Lime_FastWrite(LMS_CLK_CTRL_R, ulSavedClockCtrl);
	}
}

//*****************************************************************************
//
// Reads a configuration register and places the masked value in the passed ul buffer.
//
// \param ulRegister The LMS configuration register address to read.
// \param ulFieldMask A mask to select which bits we should read.
// \param pulBuffer A pointer to an unsigned long value to write the read value to.
//
//*****************************************************************************
int
Lime_GetRegisterField(unsigned long ulRegister, unsigned long ulFieldMask, unsigned long* pulBuffer)
{
	// Read the value
	Lime_Read(ulRegister, pulBuffer);

	// Mask out the data of interest
	*pulBuffer = *pulBuffer & ulFieldMask;

	return -1;
}

//*****************************************************************************
//
// Toggle the RXOUTSW bit. Switch closed = RXI/RXQ available on header.
//
// \return A 0 upon success or a 1 on failure.
//
//*****************************************************************************
int
Lime_ToggleRXOutSwitch(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_CLK_CTRL_R, &ulBuff);
	ulBuff ^= LMS_CLK_EN_RXOUTSW;	//toggle the RXOUTSW

	// Display what's going on.
	if (ulBuff & LMS_CLK_EN_RXOUTSW)
	{
		return Lime_CloseRXOutSwitch();
	}
	else
	{
		return Lime_OpenRXOutSwitch();
	}
}

int
Lime_OpenRXOutSwitch(void)
{
	PrintStr("RXOUTSW - open");
	__newline
	return Lime_SetRegisterField(LMS_CLK_CTRL_R, 0x00, LMS_CLK_EN_RXOUTSW);
}

int
Lime_CloseRXOutSwitch(void)
{
	PrintStr("RXOUTSW - closed");
	__newline
	return Lime_SetRegisterField(LMS_CLK_CTRL_R, 0xFF, LMS_CLK_EN_RXOUTSW);
}

//*****************************************************************************
//
// Log-Linear gain setting for RX LNA. Takes values: [0, 1, 2] <=> [0, 3, 6] dB.
// Should be linear in 3 dB increments. We need to test this.
//
// \param ulVal A gain setting between 0 and 10.
//
// \return A 0 if successful, or 1 on error.
//
//*****************************************************************************
int
Lime_SetLNAGain(unsigned long ulVal)
{
	char pcBuff[5];

	if (ulVal > 2) //can't be less than 0
	{
		// illegal value!
		return 1;
	}

	myLong2Hex(ulVal, pcBuff, 1);
	PrintStr("Set LNA Gain: ");
	PrintStr((unsigned char *)pcBuff);
	__newline;

	// Gain values 0:2 map to settings 1:3 in bits 7:6
	ulVal = (ulVal + 1) << 6;
	return Lime_SetRegisterField(LMS_LNA_CTRL_R, ulVal, LMS_GAIN_LNA_M);
}

//*****************************************************************************
//
// Toggles the Soft Reset bit of the LMS6002D and turns off all external RF PAs.
// This doesn't appear to overwrite or default any LMS6002D config registers,
// rather it just puts the chip into an "off" state. This stops PLLs, which
// require 20ms settling time on restart.
//
// \return A zero if successful, or the number of errors.
//
//*****************************************************************************
int
Lime_SoftReset_PAsOff(void)
{
	int errorsOccurred = 0;

	// Turn off the TX PAs. We're about to reset the LMS6002D and our settings
	// are no longer valid.
	WSD_TurnOffUHFPAs();
	WSD_TurnOffWiFiPAs();

	// Soft Reset = active (it's active-low)
	errorsOccurred +=
			Lime_SetRegisterField(LMS_TOP_SOFTEN_R, 0, LMS_TOP_SRESET_PD);

	// Disable Soft Reset
	errorsOccurred +=
			Lime_SetRegisterField(LMS_TOP_SOFTEN_R, LMS_TOP_SRESET_PD, LMS_TOP_SRESET_PD);

	return errorsOccurred;
}

//*****************************************************************************
//
// Set the RX I/Q DCO compensation DACs. The sign+mag register is mapped to: [0,126]
//
// Note that global variables ulRXDCO_I_Val and ulRXDCO_Q_Val are updated. This is to
// allow fast Tx/Rx switching by having local copies of these values. The startup
// default value of each register is zero.
//
// \param ulBaseAddr The address of the DCO DAC config register: LMS_DCO_RXFE_I_R, LMS_DCO_RXFE_Q_R
// \param ulDCOVal The value to set: DCO values [-63, 63] have been mapped to [0, 126]
//
// \return A 0 upon success, or a 1 if an error occurs.
//
//*****************************************************************************
int
Lime_SetRXFE_DCO(unsigned long ulBaseAddr, unsigned long ulDCOVal)
{
	unsigned long ulBuff;

	// Test the range of the input value
	if (ulDCOVal > 126)
	{
		// value is OOB.
		Stel_ThrowError("OOB DCO Value");
	}

	if (ulDCOVal >= 63)
	{
		// Values [63,126] => [0, 63]
		ulBuff = ulDCOVal - 63;
	}
	else
	{
		// Values [0,62] => [-63,-1] => BIT6 | [63,1]
		ulBuff = 63 - ulDCOVal;
		// Set sign bit to indicate odd.
		ulBuff |= BIT6;
	}

	// Save these values so that other code can quickly change RXFE_IN1SEL
	// and INLOAD_LNA_RXFE without modifying anything else.
	if (ulBaseAddr == LMS_DCO_RXFE_I_R)
	{
		trx.ulCurrentRx_I_DCO = ulBuff & LMS_DCO_RXFE_M;
	}
	else if (ulBaseAddr == LMS_DCO_RXFE_Q_R)
	{
		trx.ulCurrentRx_Q_DCO = ulBuff & LMS_DCO_RXFE_M;
	}

	// Actually set the DCO value.
	return Lime_SetRegisterField(ulBaseAddr, ulBuff, LMS_DCO_RXFE_M);
}

//*****************************************************************************
//
//	Read the current RXFE DCO DAC for I/Q and convert the SIGN+MAG to [0, 126] value
//
// \param ulBaseAddr The address of the DCO DAC config register: LMS_DCO_RXFE_I_R, LMS_DCO_RXFE_Q_R
//
// \return The RX DCO DAC setting as an unsigned long [0, 126]
//
//*****************************************************************************
unsigned long
Lime_GetRXFE_DCO(unsigned long ulBaseAddr)
{
	unsigned long ulBuff;

	Lime_Read(ulBaseAddr, &ulBuff);
	if (ulBuff & LMS_DCO_RXFE_SIGN_M)
	{
		// Maps MAG: [63, 1] => [0, 62]
		ulBuff = 63 - (ulBuff & LMS_DCO_RXFE_MAG_M);
	}
	else
	{
		// Maps MAG: [0, 63] => [63, 126]
		ulBuff = 63 + (ulBuff & LMS_DCO_RXFE_MAG_M);
	}

	return ulBuff;
}

//*****************************************************************************
//
// Set the TXRF DCO compensation DAC for VGA1. Values [0,255]=>[-16,+15.875]
//
// \param ulBaseAddr The target I/Q TX DCO register: LMS_DCO_TXRF_I_R, LMS_DCO_TXRF_Q_R
// \param ulDCOVal The TX VGA1 DCO value between [0, 255] => [-16, +15.875] mV
//
// \return A 0 upon success, or a 1 if an error occurs.
//
//*****************************************************************************
int
Lime_SetTXRF_DCO(unsigned long ulBaseAddr, unsigned long ulDCOVal)
{
	// Map [0, 255] => [-16, 15.875] mV
	//  -16        -0.125     0          0.125      15.875
	// [00000000...01111111...10000000...10000001...11111111]
	// Note that the Lime guys are awesome and this is already correct!
	return Lime_RobustWrite(ulBaseAddr, ulDCOVal);
}

//*****************************************************************************
//
// Read the current targeted TXRF DCO compensation DAC value for VGA1.
//
// \param ulBaseAddr The target I/Q TX DCO register: LMS_DCO_TXRF_I_R, LMS_DCO_TXRF_Q_R
//
// \return The DCO DAC setting for the given I/Q DAC as an unsigned long [0, 255]
//
//*****************************************************************************
unsigned long
Lime_GetTXRF_DCO(unsigned long ulBaseAddr)
{
	unsigned long ulBuff;

	Lime_Read(ulBaseAddr, &ulBuff);
	return ulBuff;
}

//*****************************************************************************
//
// USED FOR CALIBRATION ONLY!!
// Provide direct register access to the Rx LOFT Calibration registers
//
// \param ulBaseAddr The address of the DCO DAC config register: LMS_DCO_RXFE_I_R, LMS_DCO_RXFE_Q_R
// \param ulDCOVal The value to write directly to the raw register
//
// \return a zero on success
//
//*****************************************************************************
int
Lime_SetRawRXFE_DCO(unsigned long ulBaseAddr, unsigned long ulRegisterCode)
{
	// These values are saved as globals so that Tx/Rx switching can
	// quickly update RXFE_IN1SEL without changing or reading these values.
	if (ulBaseAddr == LMS_DCO_RXFE_I_R)
	{
		trx.ulCurrentRx_I_DCO = ulRegisterCode;
	}
	else if (ulBaseAddr == LMS_DCO_RXFE_Q_R)
	{
		trx.ulCurrentRx_Q_DCO = ulRegisterCode;
	}

	// Directly set the Rx LOFT DC level for calibration.
	// This contains NO translation to make the range of values smooth.
	return Lime_SetRegisterField(ulBaseAddr, ulRegisterCode, LMS_DCO_RXFE_M);
}

//*****************************************************************************
//
// Enable and configure RF Loopback Mode for calibration and testing.
// This works well; however a hard reset should be used when leaving loopback
// mode since it may be that not all settings are reset.
//
// \return A zero upon success. Non-zero otherwise.
//
//*****************************************************************************
int
Lime_StartRFLoopbackMode(void)
{
	int errors = 0;

	errors += Lime_SoftReset_PAsOff();

	errors += Lime_SoftTXEnable();
	errors += Lime_SoftRXEnable();

	// If Tx frequency is X, then to get the LOFT spike at 4 MHz,
	// the Rx frequency should be X - 4 MHz
//	errors += Lime_SetFrequency(LMS_TXPLL_BASE, ulTuningFreqKHz);
//	errors += Lime_SetFrequency(LMS_RXPLL_BASE, ulTuningFreqKHz - 4000);

	// Set TXVGA1 Gain = -7 dB (11100), TXVGA2 Gain = +25 (11111)
	// On pg. 42 of the programming and calibration guide, LMS says that:
	// "the receiver is programmed for maximum gain and the LOFT signal is
	// looped-back so that it can be samples by the LMS6002D ADC."
	// So we maximize the RX gains.
	errors += Lime_SetTXVGA1Gain(31);
	errors += Lime_SetTXVGA2Gain(25);
	errors += Lime_SetRXVGA1Gain(120);
	errors += Lime_SetRXVGA2Gain(10);

	// Disable RF and BB loopback before changing settings--we'll set later.
	errors += Lime_SelectBBLoopback(LMS_BB_LOOPB_DISABLED);
	errors += Lime_SelectRFLoopback(LMS_RF_LOOPB_DISABLED);

	// Disable TXVGA2
//	errorsOccurred += Lime_SelectActiveLNA(LMS_LNA_DISABLED);
	errors += Lime_SelectTXVGA2(LMS_TXVGA2_PA1OFF_PA2OFF);

	// Disable Test Mode, then write zeroes to magic LNA direct control register.
	// This is done in LMS's GUI for RF Loopback enable.
	errors += Lime_DirectEnableInternalLNAs();

	// Set LPF Tx/Rx filter bandwidth to: TX - 2.75 MHz, RX - 12 MHz to transmit
	// a 1 MHz complex exponential and receive the offset 5 MHz imbalance spike.
//	errors += Lime_SetFilterBandwidth(LMS_TXLPF_BASE, LMS_LPF_BW_1_375);
//	errors += Lime_SetFilterBandwidth(LMS_RXLPF_BASE, LMS_LPF_BW_6);

	// Select the appropriate RF loopback path (to LNA1 or LNA2), enable the loopb AUXPA,
	// direct disable LNAs
	errors += Lime_DirectDisableInternalLNAs();
	WSD_TurnOffUHFLNA();
	if (Lime_Rx_IsUHFBand())
	{
		errors += Lime_SelectRFLoopback(LMS_RF_LOOPB_LNA_1);
	}
	else if (Lime_Rx_IsWiFiBand())
	{
		errors += Lime_SelectRFLoopback(LMS_RF_LOOPB_LNA_2);
	}
	else
	{
		Stel_ThrowError("Unsupported Rx RFLB Freq");
	}
	Lime_SetAUXPAState(AUXPA_ON);

	// Disable the LNA--apparently direct disable doesn't work.
	// NOTE: this does NOT effect the PLL buffer selection.
//	Lime_SetRegisterField(LMS_LNA_CTRL_R, LMS_LNA_DISABLED, LMS_LNA_CTRL_LNASEL_M);

	// Set Tx/Rx Fsync
	// NOTE: this is default on initialization, so this code is redundant
//	errors += Lime_SetTXFsync();
//	errors += Lime_SetRXFsync();

	// Open the RXOUTSW in order to isolate the BB I/Q analog outputs from the ADC input
//	Lime_OpenRXOutSwitch();

	// Run the DCO autocalibration sequence
	errors += Lime_SetDefaultSettings();
	errors += Lime_AutoCalibrationSequence();

	return errors;
}

//*****************************************************************************
//
// Enable/Disable the RXFE test mode. Also enables the TXRF module if disabled.
//
// \return The number of errors.
//
//*****************************************************************************
int
Lime_SetRXFETestModeRegisterAccess(unsigned long ulSetting)
{
	int errorsOccurred = 0;

	if (ulSetting == LMS_SET)
	{
		// Enable RXFE module, and enable test mode register access
		errorsOccurred +=
				Lime_SetRegisterField(LMS_RXFE_CTRL_R,
						LMS_RXFE_TEST_MODE_ENABLE,
						LMS_RXFE_DECODE_DISABLE_M | LMS_RXFE_ENABLE_M);
	}
	else if (ulSetting == LMS_CLEAR)
	{
		// DISable RXFE module, and enable test mode register access
		errorsOccurred +=
				Lime_SetRegisterField(LMS_RXFE_CTRL_R,
						LMS_RXFE_TEST_MODE_DISABLE,
						LMS_RXFE_DECODE_DISABLE_M | LMS_RXFE_ENABLE_M);
	}
	else
	{
		errorsOccurred++;
	}

	return errorsOccurred;
}

//*****************************************************************************
//
// Directly DISable the LNAs via direct test mode register control.
//
// \return The number of errors.
//
//*****************************************************************************
int
Lime_DirectDisableInternalLNAs()
{
	PrintStr("DISabling LNAs");
	trx.directCTRL_LNAIsActive = FALSE;
	__newline
	int errorsOccurred = 0;

	// We've gotta do this in order to access the magic LMS LNA register
	errorsOccurred += Lime_SetRXFETestModeRegisterAccess(LMS_SET);
	errorsOccurred += Lime_SetRegisterField(LMS_LNA_DIRECT_R,
											LMS_LNA_DISABLE,
											LMS_LNA_DIRECT_PD_M);

	return errorsOccurred;
}

//*****************************************************************************
//
// Directly enable the LNAs via direct test mode register control.
//
// \return The number of errors.
//
//*****************************************************************************
int
Lime_DirectEnableInternalLNAs()
{
	PrintStr("ENabling LNAs");
	trx.directCTRL_LNAIsActive = TRUE;
	__newline
	int errorsOccurred = 0;

	// We've gotta do this in order to access the magic LMS LNA register
	errorsOccurred += Lime_SetRXFETestModeRegisterAccess(LMS_SET);
	errorsOccurred += Lime_SetRegisterField(LMS_LNA_DIRECT_R,
											LMS_LNA_ENABLE,
											LMS_LNA_DIRECT_PD_M);
	// As far as I can tell, once enabled, we can disable direct access.
	errorsOccurred += Lime_SetRXFETestModeRegisterAccess(LMS_CLEAR);

	return errorsOccurred;
}

//*****************************************************************************
//
//	Transceiver initialization sequence. This
//
// \return Zero if no errors occurred. Returns a positive value if error occurred.
//
//*****************************************************************************
int
Lime_XceiverPwrOnInitializationSequence(void)
{
	int errors = 0;
	int warn = 0;

	Stel_TurnOffErrorLED();

	PrintStr("=== Startup Initialization ===");
	__newline

	// Set the global signal state for normal operation; this ensures
	// that DCO autocalibrations and other operations are handled appropriately.
	trx.CalibrationMode = LMS_CAL_NORMAL;
	// Make sure loop-back mode is disabled
	errors += Lime_SetAUXPAState(AUXPA_OFF);
	errors += Lime_SelectRFLoopback(LMS_RF_LOOPB_DISABLED);
	errors += Lime_SelectBBLoopback(LMS_BB_LOOPB_DISABLED);

	// Disable baseband control of Tx/Rx switching. This makes sure we
	// always start up in a safe condition. And we'll start up with Rx
	// on ANT1 and PAs inactive, again--to be safe.
	WSD_DisableBBTxRxSwitchingCtrl();

	// Set Tx/Rx switching method and set default values for the LMS6002D.
	trx.TxRxSwitchingMode = TxRxSWMode_FDD;
	__LMS_PRIME_TRX_FDD;
	__LMS_SET_RX_FDD;

	// Open the RX I/Q output switch used for debugging
	Lime_OpenRXOutSwitch();

	// Software TX/RX enable. Note that hard enable pins must also be set
	// as well. Soft enable is required for many settings to work.
	// DSM SPI lines are recommended for normal operation.
	// Disable the PLLCLK_OUT buffer in order to reduce radiated clock noise.
	errors += Lime_SoftTXEnable();
	errors += Lime_SoftRXEnable();
	errors += Lime_SetClockCtrl(LMS_CLK_EN_RX_DSM_SPI | LMS_CLK_EN_TX_DSM_SPI, LMS_SET);
	errors += Lime_SetClockCtrl(LMS_CLK_EN_PLLCLKOUT, LMS_CLEAR);

	if (errors)
	{
		Stel_ThrowError("Init Seq [1]");
		warn = 1;
		errors = 0;
	}

	// Set some default frequency. NOTE: make sure this is called BEFORE
	// gains are set in order to not get a "OOB frequency warning".
	// This warning is okay, since we're starting up and the gain-setting
	// functions try to load calibration values each time they run.
	// Right now, this is fine, since the global Tx gain is initialized to
	// 0, a rational value, whereas global init frequency is 0. :/
	errors += Lime_SetFrequency(LMS_TXPLL_BASE, LMS_STARTUP_TXFREQ);
	errors += Lime_SetFrequency(LMS_RXPLL_BASE, LMS_STARTUP_RXFREQ);

	if (errors)
	{
		Stel_ThrowError("Init Seq [2]");
		warn = 1;
		errors = 0;
	}

	// Set default startup gains. The most important is the LNA--
	// the LNA setting is generally never touched by hardware or
	// software unless the user explicitly sets it.
	errors += Lime_SetMasterTxGain(LMS_STARTUP_TXMASTER_GAIN);

	errors += Lime_SetRXVGA1Gain(LMS_STARTUP_RXVGA1_GAIN);
	errors += Lime_SetRXVGA2Gain(LMS_STARTUP_RXVGA2_GAIN);

	errors += Lime_SetLNAGain(LMS_STARTUP_LNA_GAIN);

	// In WSD Revision Red, external LNAs drive the UHF receive chain
	// and the internal LNA could be powered down.
	if ( Lime_Rx_IsUHFBand() )
	{
		errors += Lime_SelectActiveLNA(LMS_LNA_1);
		if (trx.TRXState == RX_NORMAL)
		{
//			errors += Lime_DirectEnableInternalLNAs();
			errors += Lime_SetRxInputPath(INTERNAL_LNA);
		}
		else if (trx.TRXState == RX_BYPASS_INT_LNA)
		{
//			errors += Lime_DirectDisableInternalLNAs();
			errors += Lime_SetRxInputPath(EXTERNAL_LNA);
		}
		WSD_TurnOnUHFLNA();
	}
	else // WiFi band still uses internal LNA
	{
		if (trx.TRXState == RX_BYPASS_INT_LNA)
		{
			PrintStr("WiFi Chain Does Not Support Internal LNA Bypass Mode");
			__newline
		}
		errors += Lime_SelectActiveLNA(LMS_LNA_2);
		errors += Lime_SetRxInputPath(INTERNAL_LNA);
	}

	if (errors)
	{
		Stel_ThrowError("Init Seq [3]");
		warn = 1;
		errors = 0;
	}

	// Some default RF settings that aren't worth breaking out;
	// e.g. bias currents, etc...
	errors += Lime_SetDefaultSettings();
//	Lime_SetAUXPAState(AUXPA_OFF);

	// Set the LPF Filter Bandwidth for the TX and RX chains.
	// 5 MHz SSB = 10 MHz channel
	errors += Lime_SetFilterBandwidth(LMS_TXLPF_BASE, LMS_STARTUP_TXBW);
	errors += Lime_SetFilterBandwidth(LMS_RXLPF_BASE, LMS_STARTUP_TXBW);

	// DCO cancellation and LPF BW tuning
	if (Lime_AutoCalibrationSequence())
	{
		PrintStr("=== Re-running AutoCalibration ===");
		__newline
		errors += Lime_AutoCalibrationSequence();
	}

	if (errors)
	{
		Stel_ThrowError("Init Seq [4]");
		warn = 1;
		errors = 0;
	}

	// Select a default LNA: LNA 1 = UHF, LNA 2 = WiFi, LNA3 = AUX
	// Note that the Tx and Rx frequencies MUST be set before calling this
	// function, also, this will fail if the default frequencies are not
	// either a UHF or WiFi center frequency.
	if (Lime_Rx_IsUHFBand())
	{
		errors += Lime_SelectActiveLNA(LMS_LNA_1);
	}
	else if (Lime_Rx_IsWiFiBand())
	{
		errors += Lime_SelectActiveLNA(LMS_LNA_2);
	}
	else
	{
		// Default is LNA1
		errors += Lime_SelectActiveLNA(LMS_LNA_1);
		Stel_ThrowError("Unsupported Rx Freq for LNA sel");
	}

	// Select a default TXVGA2 PA: PA1 is UHF, PA2 is WiFi
	if (Lime_Tx_IsUHFBand())
	{
		// Enable UHF TxVGA2
		errors += Lime_SelectTXVGA2(LMS_TXVGA2_PA1ON_PA2OFF);
	}
	else if (Lime_Tx_IsWiFiBand())
	{
		// Enable WiFi TxVGA2
		errors += Lime_SelectTXVGA2(LMS_TXVGA2_PA1OFF_PA2ON);
	}
	else
	{
		// Default is both PAs OFF
		errors += Lime_SelectTXVGA2(LMS_TXVGA2_PA1OFF_PA2OFF);
		Stel_ThrowError("Unsupported Tx Freq for LNA Selection");
	}

	// Make sure that the TX/RX I/Q polarity state is know.
	// For WSD Rev 2 & 3, with WAB Rev 2 & 3, the correct setting
	// is: 1101. This was verified by looking at the LTS output for
	// a receiver and noting that the best correlation occurred at
	// this setting. The Tx settings were verified by interfacing
	// a daughter card with SORA.
	errors += Lime_SetTXFsync();
	errors += Lime_SetRXFsync();
	errors += Lime_ClearTXIQInterleave();
	errors += Lime_SetRXIQInterleave();

	// Turn on the weak UHF PA. This lets the FPGA control TX/RX
	// Note: in our current design, the FPGA provides the Stellaris with a Tx/Rx signal
	// on PB1, which generates an interrupt to allow the Stellaris to provide the
	// final commands for switching. This command is commented out because it is no longer
	// needed.
//	WSD_TurnOnUHF_PA_1();

	PrintStr("=== Initialization Done ===");
	__newline

	return errors + warn;
}

int
Lime_SetTXFsync(void)
{
	trx.txFSync = TRUE;
	PrintStr("TX Fsync = 1");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0xFF, LMS_MISC_TX_FSYNC_POL_M);
}

int
Lime_ClearTXFsync(void)
{
	trx.txFSync = FALSE;
	PrintStr("TX Fsync = 0");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0x00, LMS_MISC_TX_FSYNC_POL_M);
}

int
Lime_SetRXFsync(void)
{
	trx.rxFSync = TRUE;
	PrintStr("RX Fsync = 1");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0xFF, LMS_MISC_RX_FSYNC_POL_M);
}

int
Lime_ClearRXFsync(void)
{
	trx.rxFSync = FALSE;
	PrintStr("RX Fsync = 0");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0x00, LMS_MISC_RX_FSYNC_POL_M);
}

int
Lime_SetTXIQInterleave(void)
{
	trx.txIQInterleave = TRUE;
	PrintStr("TX Intlv = 1");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0xFF, LMS_MISC_TX_INTERLEAVE_M);
}

int
Lime_ClearTXIQInterleave(void)
{
	trx.txIQInterleave = FALSE;
	PrintStr("TX Intlv = 0");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0x00, LMS_MISC_TX_INTERLEAVE_M);
}

int
Lime_SetRXIQInterleave(void)
{
	trx.rxIQInterleave = TRUE;
	PrintStr("RX Intlv = 1");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0xFF, LMS_MISC_RX_INTERLEAVE_M);
}

int
Lime_ClearRXIQInterleave(void)
{
	trx.rxIQInterleave = FALSE;
	PrintStr("RX Intlv = 0");
	__newline;
	return Lime_SetRegisterField(LMS_MISC_CTRL_R, 0x00, LMS_MISC_RX_INTERLEAVE_M);
}

//*****************************************************************************
//
// Gets the current state of TX Fsync and flips it.
//
// \return A 0 if successful, or a 1 if failure.
//
//*****************************************************************************
int
Lime_TX_SwapIQPolarity(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_MISC_CTRL_R, &ulBuff);
	ulBuff &= LMS_MISC_TX_FSYNC_POL_M;

	if (ulBuff)
	{
		return Lime_ClearTXFsync();
	} else
	{
		return Lime_SetTXFsync();
	}
}

//*****************************************************************************
//
// Gets the current state of RX Fsync and flips it.
//
// \return A 0 if successful, or a 1 if failure.
//
//*****************************************************************************
int
Lime_RX_SwapIQPolarity(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_MISC_CTRL_R, &ulBuff);
	ulBuff &= LMS_MISC_RX_FSYNC_POL_M;

	if (ulBuff)
	{
		return Lime_ClearRXFsync();
	} else
	{
		return Lime_SetRXFsync();
	}
}

//*****************************************************************************
//
// Gets the current state of RX Interleave Mode and flips it.
//
// \return A 0 if successful, or a 1 if failure.
//
//*****************************************************************************
int
Lime_RX_SwapIQInterleave(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_MISC_CTRL_R, &ulBuff);
	ulBuff &= LMS_MISC_RX_INTERLEAVE_M;

	if (ulBuff)
	{
		return Lime_ClearRXIQInterleave();
	} else
	{
		return Lime_SetRXIQInterleave();
	}
}

//*****************************************************************************
//
// Gets the current state of TX Interleave Mode and flips it.
//
// \return A 0 if successful, or a 1 if failure.
//
//*****************************************************************************
int
Lime_TX_SwapIQInterleave(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_MISC_CTRL_R, &ulBuff);
	ulBuff &= LMS_MISC_TX_INTERLEAVE_M;

	if (ulBuff)
	{
		return Lime_ClearTXIQInterleave();
	} else
	{
		return Lime_SetTXIQInterleave();
	}
}

//*****************************************************************************
//
// Sets the state of the TXVGA2 PAs. Acceptable values as an argument:
//	LMS_TXVGA2_PA1OFF_PA2OFF
//	LMS_TXVGA2_PA1ON_PA2OFF
//	LMS_TXVGA2_PA1OFF_PA2ON
//
// \param ulArg VGA status variable. Acceptable values are above.
//
//*****************************************************************************
int
Lime_SelectTXVGA2(unsigned long ulArg)
{
	// I've checked this and it appears that it is correctly called based
	// on Tx frequency setting. REG 12/10/2013
	return Lime_SetRegisterField(LMS_TXVGA2_CTRL_R, ulArg, LMS_TXVGA2_CTRL_M);
}

//*****************************************************************************
//
// Sets the state of the Auxiliary loop-back PA.
//
// \param new state {AUXPA_ON, AUXPA_OFF}
//
//*****************************************************************************
int
Lime_SetAUXPAState(AUXPAState_t state)
{
	int errors = 0;

	if (state == AUXPA_ON)
	{
		// Turn on AUXPA: active-low
		errors += Lime_SetRegisterField(LMS_LOOPBACK_PA_R,
											0x00,
											LMS_LOOPBACK_PD_DRVAUX_M);
	}
	else if (state == AUXPA_OFF)
	{
		// Turn OFF AUXPA: active-low
		errors += Lime_SetRegisterField(LMS_LOOPBACK_PA_R,
										LMS_LOOPBACK_PD_DRVAUX_M,
										LMS_LOOPBACK_PD_DRVAUX_M);
	}
	else
	{
		Stel_ThrowError("72");
	}

	return errors;
}

//*****************************************************************************
//
// Displays the masked value within the register as a hex value.
//
// \param ulRegister the register to read
// \param ulMask defines the contiguous bit field to read
//
//*****************************************************************************
void
Lime_DisplayRegisterFieldVal(unsigned long ulRegister, unsigned long ulMask)
{
	unsigned long ulReadBuff;
	char pcBuff[4];

	// Get register value.
	Lime_Read(ulRegister, &ulReadBuff);

	// Find the position of the first non-zero in the field mask.
	while(!(ulMask & 1))
	{
		// shift both read value and mask by one
		ulReadBuff >>= 1;
		ulMask >>= 1;
	}

	// Mask out leading bits
	ulReadBuff &= ulMask;

	// Convert value into Hex string & display.
	myLong2Hex(ulReadBuff, pcBuff, 2);
	PrintStr((unsigned char *)pcBuff);
}

//*****************************************************************************
//
// Parses out the masked value within the register and returns it's value.
//
// \param ulRegister the register to read
// \param ulMask defines the contiguous bit field to read
//
// \return the read value as an unsigned long aligned to the LSB
//
//*****************************************************************************
unsigned long
Lime_GetRegisterFieldVal(unsigned long ulRegister, unsigned long ulMask)
{
	unsigned long ulReadBuff;

	// Get register value.
	Lime_Read(ulRegister, &ulReadBuff);

	// Find the position of the first non-zero in the field mask.
	// Shift to align field value to the LSB.
	while(!(ulMask & 1))
	{
		// shift both read value and mask by one
		ulReadBuff >>= 1;
		ulMask >>= 1;
	}

	// Mask out leading bits
	ulReadBuff &= ulMask;

	return ulReadBuff;
}
//*****************************************************************************
//
// Channel setting macro for writing clean code to set the Tx & Rx center frequencies.
// Since channel assignments are different in the US and the UK, this function
// accepts the "well-known" channel number, along with a channel type (US UHF,
// UK UHF, WIFI), and a channel bandwidth in order to select the correct
// channel frequency and filter bandwidth.
//
// This setting is then executed, making this a great top-level macro call.
// Note that both the TX and RX center frequencies are set with this command.
//
// Options are Type = LMS_US_UHF		// UHF channels for the Americas
//					  LMS_EU_UHF		// UHF channels for the EU & UK
//					  LMS_WIFI			// WiFi channels, global

//			   BW   = LMS_CHAN_5MHz		// pass the channel #
//				      LMS_CHAN_10MHz	// bonds 2; pass the lower channel #
//					  LMS_CHAN_20MHz	// bonds 4 (US) or 3 (EU); pass the
//										   lowest channel #
//
// 			   Channels are US_UHF = [14, 51]
//			    	   		EU_UHF = [21, 68]
//					   		WIFI   = [1 , 14]
//
// Example: Lime_SetChannel(LMS_US_UHF, 21, LMS_CHAN_10MHz)
//
// \return Zero if no errors occurred. Returns a positive value if error occurred.
//
//*****************************************************************************
int
Lime_SetChannel(unsigned long ulChanType, unsigned long ulChan, unsigned long ulChanBW_MHz)
{
	unsigned long ulCenterFreq_kHz;
	int iErrorsOccurred = 0;
	char cBuff[10];

	if (trx.verboseOutputEnabled)
	{
		myLong2Hex(ulChanType, cBuff, 2);
		PrintStr("Type: 0x");
		PrintStr((unsigned char *)cBuff);
		__newline
		myLong2Hex(ulChan, cBuff, 2);
		PrintStr("Chan: 0x");
		PrintStr((unsigned char *)cBuff);
		__newline
		myLong2Hex(ulChanBW_MHz, cBuff, 2);
		PrintStr("BandW: 0x");
		PrintStr((unsigned char *)cBuff);
		__newline
	}

	switch (ulChanType) {
		// ========================================================================
		// UHF channel assignments for Americas, South Korea, Taiwan, Philippines
		case LMS_US_UHF:
		{
			// Check US UHF channel bounds; can be between 14 and 51
			if((ulChan > 51) || (ulChan < 14)) {
				Stel_ThrowError("OOB US UHF Chan");
				return 1;
			}

			// Set center frequency based upon channel bandwidth
			switch (ulChanBW_MHz) {
				case LMS_CHAN_5MHz:
					// A 5 MHz channel is centered at the center frequency of the passed
					// channel.
					ulCenterFreq_kHz = (ulChan - 14)*6000 + 473000;
					ulChanBW_MHz = LMS_LPF_BW_2_75;
					break;
				case LMS_CHAN_10MHz:
					// A 10 MHz channel is centered at the upper frequency of the passed
					// channel, so that 5 MHz is in the lower, and 5 MHz is in the upper
					ulCenterFreq_kHz = (ulChan - 14)*6000 + 476000;
					ulChanBW_MHz = LMS_LPF_BW_6;
					break;
				case LMS_CHAN_20MHz:
					// A 20 MHz channel is centered at the upper frequency of the next
					// channel adjacent to the passed channel, so that 4 MHz is in the
					// lowest and highest (X, X+3), and 6 MHz is in the middle (X+1, X+2)
					ulCenterFreq_kHz = (ulChan - 14)*6000 + 482000;
					ulChanBW_MHz = LMS_LPF_BW_10;
					break;
				default:
					Stel_ThrowError("Unsupported BW");
					return 1;
			}
			break;
		}
		// ========================================================================
		// UHF channel assignments for the UK and Europe
		case LMS_EU_UHF:
		{
			// Check EU UHF channel bounds; can be between 21 and 68
			if((ulChan < 21) || (ulChan > 68)) {
				Stel_ThrowError("OOB EU UHF Chan");
				return 1;
			} else if (ulChan > 58) {
				PrintStr("WARN: Chan OOB 4 RFFE");
				__newline
			}

			// Set center frequency based upon channel bandwidth
			switch (ulChanBW_MHz) {
				case LMS_CHAN_5MHz:
					// A 5 MHz channel is centered at the center frequency of the passed
					// channel.
					ulCenterFreq_kHz = (ulChan - 21)*8000 + 474000;
					ulChanBW_MHz = LMS_LPF_BW_2_75;
					break;
				case LMS_CHAN_10MHz:
					// A 10 MHz channel is centered at the upper frequency of the passed
					// channel, so that 5 MHz is in the lower, and 5 MHz is in the upper
					ulCenterFreq_kHz = (ulChan - 21)*8000 + 478000;
					ulChanBW_MHz = LMS_LPF_BW_6;
					break;
				case LMS_CHAN_20MHz:
					// A 20 MHz channel is centered at the center frequency of the next
					// channel adjacent to the passed channel, so that 6 MHz is in the
					// lowest and highest (X, X+2), and 8 MHz is in the middle (X+1)
					ulCenterFreq_kHz = (ulChan - 21)*8000 + 482000;
					ulChanBW_MHz = LMS_LPF_BW_10;
					break;
				default:
					Stel_ThrowError("Unsupported BW");
					return 1;
			}
			break;
		}
		// ========================================================================
		// 802.11g channel assignments for the world
		case LMS_WIFI:
		{
			// Check 802.11 channel bounds; can be between 1 and 14 (14 is Japan-only)
			if((ulChan < 1) || (ulChan > 14)) {
				Stel_ThrowError("OOB 802.11g Chan");
				return 1;
			} else if (ulChan > 13) {
				PrintStr("WARN: Test Chan ONLY");
				__newline
				ulCenterFreq_kHz = 2472000;
			} else {
				ulCenterFreq_kHz = (ulChan - 1)*5000 + 2412000;
			}

			// Set center frequency based upon channel bandwidth
			switch (ulChanBW_MHz) {
				case LMS_CHAN_5MHz:
					ulChanBW_MHz = LMS_LPF_BW_2_75;
					break;
				case LMS_CHAN_10MHz:
					ulChanBW_MHz = LMS_LPF_BW_6;
					break;
				case LMS_CHAN_20MHz:
					ulChanBW_MHz = LMS_LPF_BW_10;
					break;
				default:
					Stel_ThrowError("Unsupported BW");
					return 1;
			}
			break;
		}
		default:
		{
			Stel_ThrowError("Unsupported ChanType");
			return 1;
		}
	} // end switch(iChanType)

	// output for feedback
	if (trx.verboseOutputEnabled)
	{
		PrintStr("Setting center freqs to 0x");
		myLong2Hex(ulCenterFreq_kHz, cBuff, 1);
		PrintStr((unsigned char *)cBuff);
		__newline
	}

	// Set the channel bandwidth by adjusting the Tx/Rx LPF bandwidths
	iErrorsOccurred += Lime_SetFilterBandwidth(LMS_TXLPF_BASE, ulChanBW_MHz);
	iErrorsOccurred += Lime_SetFilterBandwidth(LMS_RXLPF_BASE, ulChanBW_MHz);

	// Set the center frequency of the Tx/Rx PLLs. This also sets calib values
	iErrorsOccurred += Lime_SetFrequency(LMS_TXPLL_BASE, ulCenterFreq_kHz);
	iErrorsOccurred += Lime_SetFrequency(LMS_RXPLL_BASE, ulCenterFreq_kHz);

	return iErrorsOccurred;
}

//*****************************************************************************
//
// Read RX DCO compensation values from non-volatile Flash and set LMS6002D
//
//*****************************************************************************
int
WSD_SetSerialNumber(unsigned long ulSerial)
{
	char pcBuff[10];

	// Clear the commit flag. This makes sure you won't accidentally commit.
	trx.flashCommitHasBeenVerified = FALSE;

	// Load the
//	if (FlashUserSet(ulSerial, 0xAA) != 0)
//	{
//		PrintStr("!70!");
//		__newline;
//		return 1;
//	}
	HWREG(STEL_USER_REG0) = ulSerial;
	HWREG(STEL_USER_REG1) = 0xAAAAAAAA;

	myLong2Hex(ulSerial, pcBuff, 2);
	PrintStr(" In: 0x");
	PrintStr((unsigned char *)pcBuff);
	__newline
	myLong2Hex(WSD_GetSerialNumber(), pcBuff, 2);
	PrintStr("Set: 0x");
	PrintStr((unsigned char *)pcBuff);
	__newline

	// This skeleton code below is save just in case we later want to come back
	// and rewrite additional data to the other two user flash registers.

	// Non-volatile user flash registers. MSB is the RW bit
//	unsigned long ulRXDCO_I_Val = HWREG(USER_REG0) & USER_REG_MASK;
//	unsigned long ulRXDCO_Q_Val = HWREG(USER_REG1) & USER_REG_MASK;
//	unsigned long ulTXDCO_I_Val = HWREG(USER_REG2) & USER_REG_MASK;
//	unsigned long ulTXDCO_Q_Val = HWREG(USER_REG3) & USER_REG_MASK;

	return 0;
}

//*****************************************************************************
//
// Commits the current saved user flash registers. This must be called twice.
// The first time prints a warning; the second time actually commits the
// registers. PERMANENTLY FOR EVER AND FOR ALWAYS.
//
//*****************************************************************************
int
WSD_CommitSerialNumber(void)
{
	char pcBuff[10];
	unsigned long ulBuff, ulBuff2;

	// Get the current loaded number.
	ulBuff = WSD_GetSerialNumber();

	// Make sure there is actually a serial code loaded.
	if (ulBuff == STEL_USER_REG_MASK)
	{
		PrintStr("! Serial # is all-ones !");
		__newline
		return 1;
	}

	// Check if the serial has already been committed.
	FlashUserGet(&ulBuff, &ulBuff2);
	if ( (ulBuff & STEL_USER_NOTREAD_MASK) == 0 )
	{
		PrintStr("! Serial # already committed !");
		__newline
		return 1;
	}

	// Make the user commit twice before actually doing it.
	if (trx.flashCommitHasBeenVerified) {
		// Actually commit the values
		// THIS CAN'T BE UNDONE!!
		FlashUserSave();
		PrintStr("Serial # committed.");
		__newline
	} else {
		PrintStr("Current serial #: 0x");
		myLong2Hex(ulBuff & STEL_USER_REG_MASK, pcBuff, 1);
		PrintStr((unsigned char *)pcBuff);
		__newline
		PrintStr("Are you sure you want to commit the serial #?");
		__newline
		PrintStr("This CANNOT be undone. Ever. No, seriously. EVER.");
		__newline
		PrintStr("If you think this is right, then commit again w/o powering off.");
		__newline
		trx.flashCommitHasBeenVerified = TRUE;
	}
	return 0;
}

//*****************************************************************************
//
// Gets the value of the first permanent user flash register.
//
// \return The WSD serial number.
//
//*****************************************************************************
unsigned long
WSD_GetSerialNumber(void)
{
	unsigned long ulBuff, ulBuff2;

	// Get the two user configuration register values
	FlashUserGet(&ulBuff, &ulBuff2);

	// Only Flash HWREG0 contains the serial number for now.
	// Note that the last bit is always one when it's uncommitted
	return (ulBuff & STEL_USER_REG_MASK);
}

//*****************************************************************************
//
// Print the current values of the LOFT DCO configuration registers.
//
//*****************************************************************************
void
WSD_PrintCurrentLOFT_DCOSettings(void)
{
	char cBuff[10];

	// Read values from the DCO DAC settings.
	// This is required because they are register fields in a specific format.
	unsigned long ulRXDCO_I_Val = Lime_GetRXFE_DCO(LMS_DCO_RXFE_I_R);
	unsigned long ulRXDCO_Q_Val = Lime_GetRXFE_DCO(LMS_DCO_RXFE_Q_R);
	unsigned long ulTXDCO_I_Val = Lime_GetTXRF_DCO(LMS_DCO_TXRF_I_R);
	unsigned long ulTXDCO_Q_Val = Lime_GetTXRF_DCO(LMS_DCO_TXRF_Q_R);

	// Pretty print the settings.
	myLong2Hex(ulRXDCO_I_Val, cBuff, 2);
	PrintStr("Current RXDCO_I: ");
	PrintStr((unsigned char *)cBuff);
	myLong2Hex(ulRXDCO_Q_Val, cBuff, 2);
	PrintStr(", RXDCO_Q: ");
	PrintStr((unsigned char *)cBuff);
	myLong2Hex(ulTXDCO_I_Val, cBuff, 2);
	PrintStr(", TXDCO_I: ");
	PrintStr((unsigned char *)cBuff);
	myLong2Hex(ulTXDCO_Q_Val, cBuff, 2);
	PrintStr(", TXDCO_Q: ");
	PrintStr((unsigned char *)cBuff);
	__newline

	return;
}

//*****************************************************************************
//
// Enables the interrupts on PB1, PB2 used for Tx/Rx switching control.
//
// NOTE: I'm not actually sure that you CAN mask the NMI. That would... you
//       know... defeat the purpose, right?
//
//*****************************************************************************
void
WSD_EnableBBTxRxSwitchingCtrl(void)
{
	PrintStr("Baseband Tx/Rx control ENABLED.");
	__newline
	// Enable the pin interrupts
	ROM_GPIOPinIntEnable(CTRL_FPGA_BASE,
						 CTRL_FPGA_TXEN_PIN | CTRL_FPGA_NMI_PIN);
	trx.automaticTxRxSwitchingEnabled = TRUE;
}

//*****************************************************************************
//
// Disables the interrupts on PB1, PB2 used for Tx/Rx switching control.
//
// NOTE: I'm not actually sure that you CAN mask the NMI. That would... you
//       know... defeat the purpose, right?
//
//
//*****************************************************************************
void
WSD_DisableBBTxRxSwitchingCtrl(void)
{
	PrintStr("Baseband Tx/Rx control DISABLED.");
	__newline
	// DISable the pin interrupts
	ROM_GPIOPinIntDisable(CTRL_FPGA_BASE,
						 CTRL_FPGA_TXEN_PIN | CTRL_FPGA_NMI_PIN);
	trx.automaticTxRxSwitchingEnabled = FALSE;
}

//*****************************************************************************
//
// Controls the state of the IN1SEL_MIX_RXFE switch. Please see Lime docs for
// more information.
//
// \param mode the state of the switch {EXTINPUT_2_LNABUFF, LNA_2_LNABUFF}
//
// \return non-zero on error
//
//*****************************************************************************
int
Lime_Set_IN1SEL_MIX_RXFE(IN1SELState_t mode)
{
	if (mode == LNA_2_LNABUFF)
	{
		// Closing this switch (high = closed) will tie
		// the LNA output to the input of the Rx Mixer buffers
		PrintStr("LNA Output to Rx Chain");
		__newline
		return Lime_SetRegisterField(LMS_IN1SEL_MIX_R,
									 LMS_IN1SEL_MIX_RXFE_M,
									 LMS_IN1SEL_MIX_RXFE_M);
	}
	else if (mode == EXTINPUT_2_LNABUFF)
	{
		// Opening this switch (high = closed) will open the LNA output
		// and tie the Rx Mixer Buff to IEXMIX1 pin. I assume this is a
		// test fixture rather than used for production, but we can use
		// it for our purposes.
		PrintStr("LNA Output Floating");
		__newline
		return Lime_SetRegisterField(LMS_IN1SEL_MIX_R,
							  	  	 0x00,
							  	  	 LMS_IN1SEL_MIX_RXFE_M);
	}
	else
	{
		Stel_ThrowError("71");
		return 1;
	}
}


//*****************************************************************************
// Sets the state of the RINEN_MIX_RXFE load.
//
// \param mode the state of the RINEN_MIX_RXFE load {LOAD_ENABLED, LOAD_DISABLED}
// \return non-zero on error
//*****************************************************************************
int
Lime_Set_RINEN_MIX_RXFE(RINEN_MIX_RXFE_t mode)
{
	if (mode == LOAD_ENABLED)
	{
		PrintStr("RINEN_MIX_RXFE Active");
		__newline
		return Lime_SetRegisterField(LMS_MIX_RXFE_R,
									 0xFF,
									 LMS_RINEN_MIX_RXFE_M);
	}
	else if (mode == LOAD_DISABLED)
	{
		PrintStr("RINEN_MIX_RXFE Active");
		__newline
		return Lime_SetRegisterField(LMS_MIX_RXFE_R,
									 0x00,
									 LMS_RINEN_MIX_RXFE_M);
	}
	else
	{
		Stel_ThrowError("72");
	}

	return 1;
}

//*****************************************************************************
// Sets the appropriate RXFE input path and related settings.
//
// Options: EXTERNAL_LNA, INTERNAL_LNA
//
// As of now, there are several modes: WSD Rev 2 used internal LNAs for
// both UHF and WiFi. But WSD Rev 3 uses an external LNA as input for the UHF
// pathway.
// Set this function when the frequency band changes or during initialization
// to ensure internal parameters get updated properly.
//*****************************************************************************
int
Lime_SetRxInputPath(RFInputSrc_t mode)
{
	int errors = 0;

	if (mode == EXTERNAL_LNA)
	{
		errors += Lime_DirectDisableInternalLNAs();
		// Enable the RINEN_MIX_RXFE load on the IEXMIX1 pad input used for
		// external LNAs.
		Lime_Set_RINEN_MIX_RXFE(LOAD_ENABLED);
		// Connect the external IEXMIX1 pads directly to the LNA Rx Buffer
		Lime_Set_IN1SEL_MIX_RXFE(EXTINPUT_2_LNABUFF);
	}
	else if (mode == INTERNAL_LNA)
	{
		// Disable the RINEN_MIX_RXFE load on the IEXMIX1 pad input
		Lime_Set_RINEN_MIX_RXFE(LOAD_DISABLED);
		// Connect the LMS internal LNA directly to the LNA Rx Buffer
		Lime_Set_IN1SEL_MIX_RXFE(LNA_2_LNABUFF);
		errors += Lime_DirectEnableInternalLNAs();
	}
	else
	{
		Stel_ThrowError("74");
		return -1;
	}
	// Save the value so other functions behave properly
	trx.RxInputMode = mode;
	return errors;
}

//*****************************************************************************
// \return A boolean-values integer indicating if Rx is within the frequency range.
//*****************************************************************************
int
Lime_Rx_IsWiFiBand(void)
{
	if (trx.ulCurrentRxFreq < 2400000 || trx.ulCurrentRxFreq > 2500000)
	{
		return 0;
	}
	return 1;
}

//*****************************************************************************
// \return A boolean-values integer indicating if Rx is within the frequency range.
//*****************************************************************************
int
Lime_Rx_IsUHFBand(void)
{
	if (trx.ulCurrentRxFreq < 470000 || trx.ulCurrentRxFreq > 773000)
	{
		return 0;
	}
	return 1;
}

//*****************************************************************************
// \return A boolean-values integer indicating if Tx is within the frequency range.
//*****************************************************************************
int
Lime_Tx_IsWiFiBand(void)
{
	if (trx.ulCurrentTxFreq < 2400000 || trx.ulCurrentTxFreq > 2500000)
	{
		return 0;
	}
	return 1;
}

//*****************************************************************************
// \return A boolean-values integer indicating if Tx is within the frequency range.
//*****************************************************************************
int
Lime_Tx_IsUHFBand(void)
{
	if (trx.ulCurrentTxFreq < 470000 || trx.ulCurrentTxFreq > 773000)
	{
		return 0;
	}
	return 1;
}

//*****************************************************************************
// Runs the calibration algorithm for the RC-setting for Tx and Rx LPFs.
//
// Lime claims that this value should almost always be 3, but it's nice to be
// able to detect and correct for any chip that falls far out of spec.
//
// We had a lot of trouble getting this to work. The actual sequence of commands
// was eventually reverse-engineered from documentation, USB proto analysis, and
// sheer trial and error. I've wasted at least 12 hours straight of my
// life on this guy.
//
// Note that Lime also claims that the calibration bandwidth doesn't matter,
// but who knows? So I'm gonna parametrize this.
//
// \param ulLPFBandwidthCode the channel bandwidth to calibrate this board to
//*****************************************************************************
int
Lime_NewLPFBandwidthTuning(unsigned long ulLPFBandwidthCode)
{
	unsigned long ulRCCAL;
	int errors = 0;
	char pcBuff[6];

	// If the PLL reference clock is not 40 MHz, then we have to calibrate the module
	// with an internally-generated reference clock. In the Lime GUI, this triggers
	// a flurry of commands that we don't know what to do with--there isn't that much
	// transparency with this. Luckily, we provide a 40 MHz reference clock
	// for the radio daughter-card, so we don't have to worry about this.
	if (LMS_PLL_REF_CLK_RATE != 40000)
	{
		return 1;
	}

	// Step 0: Enable LPF CAL Clock, RX and Tx LPF DCCal Clocks (0x09, bit 5)
	Lime_SetClockCtrl(LMS_CLK_EN_LPFCAL | LMS_CLK_EN_RX_LPF_DCCAL | LMS_CLK_EN_TX_LPF_DCCAL,
					  LMS_SET);

	// Step 0.1: ENABLE LPFCal Block EN_CAL_LPFCAL = 0 (LPF Cal Block DISabled)
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R,
									LMS_LPFCAL_2_EN_CAL_M,
									LMS_LPFCAL_2_EN_CAL_M );

	// Step 1:  Set CLKSEL_LPFCAL = 0 (40 MHz Clock)
	//			Set PD_CLKLPFCAL = 1 (power down on-chip LPF tuning clock) //
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_1_R,
							LMS_LPFCAL_1_CLKSEL_M | LMS_LPFCAL_1_PD_CLK_M,
							(LMS_LPFCAL_1_CLKSEL_M | LMS_LPFCAL_1_PD_CLK_M) );

	// Step 1.1: Ensure everything needed is enabled. RESET = 0, Tx/Rx = enabled, Top = enabled
	errors += Lime_SetRegisterField(LMS_TOP_SOFTEN_R,
			LMS_TOP_SRESET_PD | LMS_TOP_EN | LMS_TOP_STXEN | LMS_TOP_SRXEN,
			LMS_TOP_SRESET_PD | LMS_TOP_EN | LMS_TOP_STXEN | LMS_TOP_SRXEN);

	// Step 1.2: Set LPFCal bandwidth
	//		   Set BWC_LPDCal = ulLPFBandwidthCode
	//		  (calibration bandwidth given as argument to calibration)
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R,
								ulLPFBandwidthCode,
								LMS_LPFCAL_2_BWC_M );

	// Step 3: Pulse RST_CAL_LPFCAL by setting to 1 (reset)
	//		   Then Set to 0 (normal).
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_1_R,
							LMS_LPFCAL_1_RST_CAL_M,
							LMS_LPFCAL_1_RST_CAL_M );
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_1_R,
							0x00,
							LMS_LPFCAL_1_RST_CAL_M );

	// Pulse EN_CAL_LPFCAL
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R,
									LMS_LPFCAL_2_EN_CAL_M,
								    LMS_LPFCAL_2_EN_CAL_M );
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R,
								    0x00,
								    LMS_LPFCAL_2_EN_CAL_M );

	// Step 4: Read the RCCAL_LPFCAL[2:0] value (RC time constant, I guess)
	//		   Print the results.
	Lime_Read(LMS_TOP_RCCAL_R, &ulRCCAL);
	ulRCCAL = (ulRCCAL & LMS_RCCAL_LPFCAL_M) >> 5;
	PrintStr("LPFCAL Value: ");
	Stel_PrintReg(ulRCCAL, pcBuff);
	__newline

//	PrintStr("DEBUG 0x01: ");
//	Stel_PrintReg_DEBUG(LMS_TOP_RCCAL_R);
//	__newline

	// Step 5: Write the RCCAL_LPFCAL value to RCCAL_LPF on Tx and Rx LPF Modules (Bit[6,4])
	errors += Lime_SetRegisterField((LMS_TXLPF_BASE + LMS_RCCAL_OFFSET),
							 ulRCCAL << 4,
							 LMS_RCCAL_LPF_M);
	errors += Lime_SetRegisterField((LMS_RXLPF_BASE + LMS_RCCAL_OFFSET),
							 ulRCCAL << 4,
							 LMS_RCCAL_LPF_M);

//	PrintStr("DEBUG 0x36: ");
//	Stel_PrintReg_DEBUG(0x36);
//	__newline
//	PrintStr("DEBUG 0x56: ");
//	Stel_PrintReg_DEBUG(0x56);
//	__newline

	// Step 6: Put block in reset PD_CLK_LPFCAL = 1 (reset)
	//		   Disable Block EN_CAL_LPFCAL = 0 (LPF Cal Block Disabled)
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_1_R,
							LMS_LPFCAL_1_RST_CAL_M,
							LMS_LPFCAL_1_RST_CAL_M);
	errors += Lime_SetRegisterField(LMS_TOP_LPFCAL_2_R,
							0x00,
							LMS_LPFCAL_2_EN_CAL_M);

	// Step 7: Disable LPFCAL Block Clock
	errors += Lime_SetClockCtrl(LMS_CLK_EN_LPFCAL, LMS_CLEAR);

	return errors;
}

//*****************************************************************************
//TODO
//*****************************************************************************
int
Lime_FuckingNukeEverything(void)
{
	Lime_SetRegisterField(0x09, B8(10000000), B8(11111111));
	Lime_SetRegisterField(0x0B, B8(00010010), B8(00010011));

	Lime_SetRegisterField(0x15, B8(00000000), B8(11111100));
	Lime_SetRegisterField(0x25, B8(00010010), B8(00010011));

	Lime_SetRegisterField(0x17, B8(01000000), B8(01000000));
	Lime_SetRegisterField(0x27, B8(01000000), B8(01000000));

	Lime_SetRegisterField(0x1B, B8(00001000), B8(00001000));
	Lime_SetRegisterField(0x2B, B8(00001000), B8(00001000));

	Lime_SetRegisterField(0x34, B8(00000000), B8(00000010));
	Lime_SetRegisterField(0x36, B8(10000111), B8(10000111));
	Lime_SetRegisterField(0x3F, B8(10000000), B8(10000000));

	Lime_SetRegisterField(0x54, B8(00000010), B8(00000010));
	Lime_SetRegisterField(0x56, B8(00000111), B8(00000111));
	Lime_SetRegisterField(0x57, B8(10000000), B8(10000000)); // This is the ADC/DAC register, may require futzing
	Lime_SetRegisterField(0x59, B8(00000000), B8(00000001));
	Lime_SetRegisterField(0x5F, B8(10000000), B8(10000000));

	Lime_SetRegisterField(0x40, B8(00000000), B8(00000010));
	Lime_SetRegisterField(0x44, B8(00000100), B8(00011100));
	Lime_SetRegisterField(0x4A, B8(00000111), B8(00011111));
	Lime_SetRegisterField(0x4E, B8(10000000), B8(10000000));

	Lime_SetRegisterField(0x64, B8(00000000), B8(00000010));
	Lime_SetRegisterField(0x6E, B8(11000000), B8(11000000));

	Lime_SetRegisterField(0x70, B8(00000000), B8(00000001));
	Lime_SetRegisterField(0x75, B8(00000000), B8(00110000));
	Lime_SetRegisterField(0x7D, B8(00001111), B8(00001111));

	return 0;
}

//*****************************************************************************
//TODO
//*****************************************************************************
int Lime_FlipDACClkEdgePolarity(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_MISC_CTRL_R, &ulBuff);
	if (ulBuff & LMS_MISC_DAC_CLK_EDGE_POL_M)
	{
		// Clear
		Lime_SetRegisterField(LMS_MISC_CTRL_R, 0x00, LMS_MISC_DAC_CLK_EDGE_POL_M);
		trx.dacClkEdgePolarity = FALSE;
	}
	else
	{
		// Set
		Lime_SetRegisterField(LMS_MISC_CTRL_R, 0xFF, LMS_MISC_DAC_CLK_EDGE_POL_M);
		trx.dacClkEdgePolarity = TRUE;
	}

	return 1;
}

//*****************************************************************************
//TODO
//*****************************************************************************
int Lime_FlipADCSampPhaseSel(void)
{
	unsigned long ulBuff;

	Lime_Read(LMS_MISC_CTRL_R, &ulBuff);
	if (ulBuff & LMS_MISC_ADC_SAMPLE_PHASE_SEL_M)
	{
		// Clear
		Lime_SetRegisterField(LMS_MISC_CTRL_R, 0x00, LMS_MISC_ADC_SAMPLE_PHASE_SEL_M);
		trx.adcPhaseSel = FALSE;
	}
	else
	{
		// Set
		Lime_SetRegisterField(LMS_MISC_CTRL_R, 0xFF, LMS_MISC_ADC_SAMPLE_PHASE_SEL_M);
		trx.adcPhaseSel = TRUE;
	}

	return 1;
}

//*****************************************************************************
//TODO
//*****************************************************************************
int Lime_SetClkNonOverlapPhaseAdj(unsigned long new_index)
{
	unsigned long ulBuff;

	Lime_Read(LMS_MISC_CTRL_R, &ulBuff);

	Lime_SetRegisterField(LMS_MISC_CTRL_R,
						((ulBuff & LMS_MISC_ADC_SAMPLE_PHASE_SEL_M) + 1) % 3,
						LMS_MISC_ADC_SAMPLE_PHASE_SEL_M);
	trx.clkNonOverlapAdj = (char)((ulBuff & LMS_MISC_ADC_SAMPLE_PHASE_SEL_M) + 1) % 3;

	return 1;
}

//*****************************************************************************
//TODO
//*****************************************************************************
int
Lime_StartBBLoopbackMode(void)
{
	int errors = 0;

		errors += Lime_SoftReset_PAsOff();

		errors += Lime_SoftTXEnable();
		errors += Lime_SoftRXEnable();

		// Disable RF and BB loopback before changing settings--we'll set later.
		errors += Lime_SelectBBLoopback(LMS_BB_LOOPB_DISABLED);
		errors += Lime_SelectRFLoopback(LMS_RF_LOOPB_DISABLED);

		// Disable TXVGA2
	//	errorsOccurred += Lime_SelectActiveLNA(LMS_LNA_DISABLED);
		errors += Lime_SelectTXVGA2(LMS_TXVGA2_PA1OFF_PA2OFF);

		// Disable Test Mode, then write zeroes to magic LNA direct control register.
		// This is done in LMS's GUI for RF Loopback enable.
		errors += Lime_DirectEnableInternalLNAs();

		// Set LPF Tx/Rx filter bandwidth to 20 MHz.
		errors += Lime_SetFilterBandwidth(LMS_TXLPF_BASE, LMS_LPF_BW_10);
		errors += Lime_SetFilterBandwidth(LMS_RXLPF_BASE, LMS_LPF_BW_10);

		// Select the appropriate RF loopback path (to LNA1 or LNA2), enable the loopb AUXPA,
		// direct disable LNAs
		errors += Lime_DirectDisableInternalLNAs();
		WSD_TurnOffUHFLNA();

		// Select which of the BB LoopB paths are active.
		// These paths should probably be mutually exclusive, or so that is how
		// I interpreted it.
		errors += Lime_SelectBBLoopback(LMS_BB_LOOPB_OPIN);

		// As per protocol analyzer results, disable TOP modules
		errors += Lime_SetRegisterField(LMS_RXFE_CTRL_R,
										0x00,
										LMS_RXFE_ENABLE_M);

		// Should close the LoopBB switches. Set to LOOPBBEN = 01
		errors += Lime_SetRegisterField(LMS_BB_LOOPB_R,
										0x04,
										LMS_LOOPBBEN_M);

		// Set DAC output current to only 2.5 ma to reduce BBLoopB
		// magnitude at the ADC.
		errors += Lime_SetRegisterField(LMS_DAC_CTRL_R,
										0x02,
										LMS_TX_CTRL_DAC_CURR_M);

		// Enable ADC Input Buffer to further reduce the input voltage level to
		// within the range of the ADC.
		errors += Lime_SetRegisterField(LMS_ADC_CTRL_2_R,
										0x00,
										LMS_ADC_IN_BUFF_DISABLE_M);

		// Open the RXOUTSW in order to isolate the BB I/Q analog outputs from the ADC input
	//	Lime_OpenRXOutSwitch();

		// Run the DCO autocalibration sequence
//		errors += Lime_SetDefaultSettings();
//		errors += Lime_AutoCalibrationSequence();

		return errors;
}


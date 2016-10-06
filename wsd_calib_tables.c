/*
 * calib_tables.c
 *
 *  Created on: Feb 22, 2014
 *      Author: rng
 */

#include "include/wsd_calib_tables.h"

// Included so that we could use TI's flash_pb driver as a template internally.
// We can't use their actual driver anymore because we now have two calibration
// tables and parameter block buffers to handle, while their driver handles
// everything with a bunch of global parameters. BAD TI.
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"

// Global calibration table variables.
//Do not access directly--use get/set methods.
//CalTable_t g_CalTable;
UHFCalTable_t 	g_UHFCalTable;
WiFiCalTable_t 	g_WiFiCalTable;


//*****************************************************************************
//
// Initialize the persistent FLASH calibration table driver & copy into SRAM.
//
// \return A zero upon success. Non-zero on error.
//
//*****************************************************************************
int
Stel_InitFlashCalibrationTables(void)
{
	unsigned char * pucBuff;

//	char pcBuff[10];

//	char cBuff[10];
//	PrintStrUARTDebug("Tablesize HEX ");
//	myLong2Hex(sizeof(tCalib_DCO_LOFT_Table), cBuff, 1);
//	PrintStrUARTDebug((unsigned char *)cBuff);
//	PrintStrUARTDebug("\n\r");
//	PrintStrUARTDebug("Rowsize HEX ");
//	myLong2Hex(sizeof(tCalib_DCO_LOFT_Row), cBuff, 1);
//	PrintStrUARTDebug((unsigned char *)cBuff);
//	PrintStrUARTDebug("\n\r");

	// Initialize the UHF calibration table by searching through flash for a
	// legit parameter block
	pucBuff = Stel_FlashPBInit(FLASH_PB_START_UHF, FLASH_PB_END_UHF, sizeof(UHFCalTable_t));
	if (pucBuff == NULL)
	{
		return 1;
	}

//	PrintStr("\r\n UHF Init pucBuff: ");
//	Stel_PrintReg((uint32_t)pucBuff, pcBuff);

	// Copy the flash table to the SRAM table. Now it's initialized.
	// This is done so that the table can be mutable, and then later can be saved
	// back to flash.
	if (Stel_SetUHFCalibrationTable((UHFCalTable_t *)pucBuff))
	{
		return 1;
	}

	// Initialize the WiFi calibration table by searching through flash for a
	// legit parameter block
	pucBuff = Stel_FlashPBInit(FLASH_PB_START_WIFI, FLASH_PB_END_WIFI, sizeof(WiFiCalTable_t));
	if (pucBuff == NULL)
	{
		return 1;
	}

//	PrintStr("\r\n WiFi Init pucBuff: ");
//	Stel_PrintReg((uint32_t)pucBuff, pcBuff);

	// Point the global table to the SRAM table. Now it's initialized.
	// This is done so that the table can be mutable, and then later can be saved
	// back to flash.
	if (Stel_SetWiFiCalibrationTable((WiFiCalTable_t *)pucBuff))
	{
		return 1;
	}

	return 0;
}

//*****************************************************************************
//
// Returns a pointer to the SRAM calibration table.
//
// \return A pointer to the mutable calibration table. NULL on error or bad table.
//
//*****************************************************************************
UHFCalTable_t *
Stel_GetUHFCalibrationTable(void)
{
	// Test for a non-initialized flash table or something wrong with it.
//	if (psCalibrationTable == NULL)
//	{
////		PrintStr("Returning NULL Table");
//		PrintStrUARTDebug("! NULL Table 1\n\r");
//		__newline
//		return NULL;
//	}

	// Our work here is done.
	return &g_UHFCalTable;
}

WiFiCalTable_t *
Stel_GetWiFiCalibrationTable(void)
{
	// Test for a non-initialized flash table or something wrong with it.
//	if (psCalibrationTable == NULL)
//	{
////		PrintStr("Returning NULL Table");
//		PrintStrUARTDebug("! NULL Table 1\n\r");
//		__newline
//		return NULL;
//	}

	// Our work here is done.
	return &g_WiFiCalTable;
}

//*****************************************************************************
//
// Copy the Flash parameter block to the SRAM calibration table, thus making it
// mutable. To make the new table non-volatile, you will have to commit the SRAM
// table back to flash.
//
// This function is hard-coded to the UHF calibration table.
//
// \return A zero upon success. Non-zero on error.
//
//*****************************************************************************
int
Stel_SetUHFCalibrationTable(UHFCalTable_t * psNewTable)
{
	// Test for a non-initialized flash table or something wrong with it.
	if (psNewTable == NULL)
	{
		return 1;
	}

	// Otherwise, set the global table to the new table
	memcpy(&g_UHFCalTable, psNewTable, sizeof(UHFCalTable_t));

	return 0;
}

//*****************************************************************************
//
// Copy the Flash parameter block to the SRAM calibration table, thus making it
// mutable. To make the new table non-volatile, you will have to commit the SRAM
// table back to flash.
//
// This function is hard-coded to the WiFi calibration table.
//
// \return A zero upon success. Non-zero on error.
//
//*****************************************************************************
int
Stel_SetWiFiCalibrationTable(WiFiCalTable_t * psNewTable)
{
	// Test for a non-initialized flash table or something wrong with it.
	if (psNewTable == NULL)
	{
		return 1;
	}

	// Otherwise, set the global table to the new table
	memcpy(&g_WiFiCalTable, psNewTable, sizeof(WiFiCalTable_t));

	return 0;
}

//*****************************************************************************
//
// Overwrite the calibration table (if any) stored in FLASH with the current
// one in SRAM. Error checking included.
//
// Because of the fault-tolerant nature of the flashpb driver library, this is
// an atomic operation. Although it's not perfect because only one FLASH page
// is currently allocated. For perfect fault-tolerance, we'd have to allocate
// TWO pages.
//
// \return A zero upon success. Non-zero on error.
//
//*****************************************************************************
int
Stel_AtomicCommitCalibrationTable()
{
	int errors = 0;
	// Test for a non-initialized flash table or something wrong with it.
//	if (psCalibrationTable == NULL)
//	{
//		PrintStr("! Can't Commit NULL Table !");
//		PrintStrUARTDebug("! Can't Commit Table\n\r");
//		__newline
//		return 1;
//	}

	// Write the current global calibration table to FLASH.
	errors += Stel_FlashPBSave((unsigned char *)&g_UHFCalTable, CAL_BAND_INDEX_UHF);

//	if (errors){PrintStr("\r\nUHF Save Failed"); errors = 0;}

	errors += Stel_FlashPBSave((unsigned char *)&g_WiFiCalTable, CAL_BAND_INDEX_WIFI);

//	if (errors){PrintStr("\r\nWiFi Save Failed"); errors = 0;}

	// Reload the SRAM table in order to update checksum and seqno.
	errors += Stel_InitFlashCalibrationTables();

	return errors;
}

//*****************************************************************************
//
// Print the current RF Calibration Table contents to the terminal.
//
// \param band_no integer band index to print starting at index 0. Index -1 means print
//                all tables.
//
//*****************************************************************************
void
Stel_PrintCalibrationTable(int band_no)
{
	char pcBuff[10];
#define NUMCOLS 9			//programmatic hack.
#define MAXVAL 25			// hack 2

	uint32_t uiBand;
	uint32_t uiFrequency;
	uint32_t uiIndex;

	// I'm actually not sure this check does anything anymore. since we're
	// hard-coding the tables and they're automatically initialized on startup.
	// Instead, we should probably check for bad seqno or checksum values.
	if (&g_UHFCalTable == NULL || &g_WiFiCalTable == NULL)
	{
		Stel_ThrowError("Cal Table Uninitialized");
		return;
	}

	// Print table header w/ table number
	PrintStr("======= Tables ");
	myLong2Hex((unsigned long)g_UHFCalTable.ucSeqNo, pcBuff, 3);
	PrintStr((unsigned char *)pcBuff);
	PrintStr(", ");
	myLong2Hex((unsigned long)g_WiFiCalTable.ucSeqNo, pcBuff, 3);
		PrintStr((unsigned char *)pcBuff);
	PrintStr(" === Size ");
	myLong2Hex((unsigned long)sizeof(UHFCalTable_t), pcBuff, 4);
	PrintStr((unsigned char *)pcBuff);
	PrintStr(", ");
	myLong2Hex((unsigned long)sizeof(WiFiCalTable_t), pcBuff, 4);
		PrintStr((unsigned char *)pcBuff);
	PrintStr(" =======");
	__newline

	if ((band_no == -1)||(band_no == 0))
	{
		uiBand = 0;

		// Print band header w/ band number
		PrintStr(" ==================== Band 00 ====================");
		__newline
		// Print the RX LOFT Values
//		PrintStr("RX_I RX_Q");
//		__newline
//		PrintStr("0x");
//		myLong2Hex(g_UHFCalTable.rx_loft_cal.i, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(" 0x");
//		myLong2Hex(g_UHFCalTable.rx_loft_cal.q, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		__newline
		// Print the TX LOFT Values in a 2-column table
		PrintStr("GAIN TX_I TX_Q   GAIN TX_I TX_Q   GAIN TX_I TX_Q");
		__newline
		for (uiIndex = 0; uiIndex < NUMCOLS; uiIndex++)
		{
			// Print Tx LOFT setting for [0-Num/3]
			PrintStr(" ");
			myLong2Hex(uiIndex, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("  0x");
			myLong2Hex(g_UHFCalTable.tx_loft_cal[uiIndex].i, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(" 0x");
			myLong2Hex(g_UHFCalTable.tx_loft_cal[uiIndex].q, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);

			// Print Tx LOFT setting for [Num/3, Num*2/3]
			PrintStr("    ");
			myLong2Hex(uiIndex + NUMCOLS, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("  0x");
			myLong2Hex(g_UHFCalTable.tx_loft_cal[uiIndex + NUMCOLS].i,
					pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(" 0x");
			myLong2Hex(g_UHFCalTable.tx_loft_cal[uiIndex + NUMCOLS].q,
					pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);

			// Print Tx LOFT setting for [Num*2/3, Num]
			// Don't print OOB values if the columns aren't equal.
			if (uiIndex + 2*NUMCOLS <= MAXVAL) //RYAN
			{
				PrintStr("    ");
				myLong2Hex(uiIndex + 2*NUMCOLS, pcBuff, 2);
				PrintStr((unsigned char *)pcBuff);
				PrintStr("  0x");
				myLong2Hex(g_UHFCalTable.tx_loft_cal[uiIndex + 2*NUMCOLS].i,
						pcBuff, 2);
				PrintStr((unsigned char *)pcBuff);
				PrintStr(" 0x");
				myLong2Hex(g_UHFCalTable.tx_loft_cal[uiIndex + 2*NUMCOLS].q,
						pcBuff, 2);
				PrintStr((unsigned char *)pcBuff);
			}
			__newline
		}

		// Print the TX IQ Imbalance values
		PrintStr("FREQUENCY   TX_SINMULT  TX_COSMULT  TX_GAINMULT");
//		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000");
		__newline
		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
		{
			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
			Stel_PrintReg(uiFrequency, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_UHFCalTable.tx_iq_bb_cal[uiIndex].smult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_UHFCalTable.tx_iq_bb_cal[uiIndex].cmult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_UHFCalTable.tx_iq_bb_cal[uiIndex].gmult, pcBuff);
			__newline
		}

		// Print the RX IQ Imbalance values
		PrintStr("FREQUENCY   RX_SINMULT  RX_COSMULT  RX_GAINMULT RX_I RX_Q");
//		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000 0x00 0x00");
		__newline
		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
		{
			// Rx IQ Imbalance Compensation Values
			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
			Stel_PrintReg(uiFrequency, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_UHFCalTable.rx_iq_bb_cal[uiIndex].smult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_UHFCalTable.rx_iq_bb_cal[uiIndex].cmult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_UHFCalTable.rx_iq_bb_cal[uiIndex].gmult, pcBuff);

			// Rx LOFT Calibration Values
			PrintStr(" 0x");
			myLong2Hex(g_UHFCalTable.rx_loft_cal[uiIndex].i, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(" 0x");
			myLong2Hex(g_UHFCalTable.rx_loft_cal[uiIndex].q, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);

			__newline
		}
	}

	if ((band_no == -1)||(band_no == 1))
	{
		uiBand = 1;

		// Print band header w/ band number
		PrintStr(" ==================== Band 01 ====================");
		__newline
		// Print the RX LOFT Values
//		PrintStr("RX_I RX_Q");
//		__newline
//		PrintStr("0x");
//		myLong2Hex(pCalTable->WiFiTable.rx_loft_cal.i, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		PrintStr(" 0x");
//		myLong2Hex(pCalTable->WiFiTable.rx_loft_cal.q, pcBuff, 2);
//		PrintStr((unsigned char *)pcBuff);
//		__newline
		// Print the TX LOFT Values in a 2-column table
		PrintStr("GAIN TX_I TX_Q   GAIN TX_I TX_Q   GAIN TX_I TX_Q");
		__newline
		for (uiIndex = 0; uiIndex < NUMCOLS; uiIndex++)
		{
			// Print Tx LOFT setting for [0-Num/3]
			PrintStr(" ");
			myLong2Hex(uiIndex, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("  0x");
			myLong2Hex(g_WiFiCalTable.tx_loft_cal[uiIndex].i, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(" 0x");
			myLong2Hex(g_WiFiCalTable.tx_loft_cal[uiIndex].q, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);

			// Print Tx LOFT setting for [Num/3, Num*2/3]
			PrintStr("    ");
			myLong2Hex(uiIndex + NUMCOLS, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr("  0x");
			myLong2Hex(g_WiFiCalTable.tx_loft_cal[uiIndex + NUMCOLS].i,
					pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(" 0x");
			myLong2Hex(g_WiFiCalTable.tx_loft_cal[uiIndex + NUMCOLS].q,
					pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);

			// Print Tx LOFT setting for [Num*2/3, Num]
			// Don't print beyond MAXVAL if columns aren't even
			if (uiIndex + 2*NUMCOLS <= MAXVAL) //RYAN
			{
				PrintStr("    ");
				myLong2Hex(uiIndex + 2*NUMCOLS, pcBuff, 2);
				PrintStr((unsigned char *)pcBuff);
				PrintStr("  0x");
				myLong2Hex(g_WiFiCalTable.tx_loft_cal[uiIndex + 2*NUMCOLS].i,
						pcBuff, 2);
				PrintStr((unsigned char *)pcBuff);
				PrintStr(" 0x");
				myLong2Hex(g_WiFiCalTable.tx_loft_cal[uiIndex + 2*NUMCOLS].q,
						pcBuff, 2);
				PrintStr((unsigned char *)pcBuff);
			}
			__newline
		}

		// Print the TX IQ Imbalance values
		PrintStr("FREQUENCY   TX_SINMULT  TX_COSMULT  TX_GAINMULT");
//		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000");
		__newline
		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
		{
			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
			Stel_PrintReg(uiFrequency, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_WiFiCalTable.tx_iq_bb_cal[uiIndex].smult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_WiFiCalTable.tx_iq_bb_cal[uiIndex].cmult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_WiFiCalTable.tx_iq_bb_cal[uiIndex].gmult, pcBuff);
			__newline
		}

		// Print the RX IQ Imbalance values
		PrintStr("FREQUENCY   RX_SINMULT  RX_COSMULT  RX_GAINMULT RX_I RX_Q");
//		PrintStr("0x0000.0000 0x0000.0000 0x0000.0000 0x0000.0000 0x00 0x00");
		__newline
		for (uiIndex = 0; uiIndex < CAL_NUM_FREQ_POINTS[uiBand]; uiIndex++)
		{
			uiFrequency = CAL_LOWER_KHZ[uiBand] + uiIndex*CAL_STEP_KHZ[uiBand];
			Stel_PrintReg(uiFrequency, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_WiFiCalTable.rx_iq_bb_cal[uiIndex].smult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_WiFiCalTable.rx_iq_bb_cal[uiIndex].cmult, pcBuff);
			PrintStr(" ");
			Stel_PrintReg(g_WiFiCalTable.rx_iq_bb_cal[uiIndex].gmult, pcBuff);

			PrintStr(" 0x");
			myLong2Hex(g_WiFiCalTable.rx_loft_cal[uiIndex].i, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			PrintStr(" 0x");
			myLong2Hex(g_WiFiCalTable.rx_loft_cal[uiIndex].q, pcBuff, 2);
			PrintStr((unsigned char *)pcBuff);
			__newline
		}
	}

	// Done printing!
	return;
}

//*****************************************************************************
//
//! Initializes the flash parameter block.
//!
//! \param ulStart is the address of the flash memory to be used for storing
//! flash parameter blocks; this must be the start of an erase block in the
//! flash.
//! \param ulEnd is the address of the end of flash memory to be used for
//! storing flash parameter blocks; this must be the start of an erase block in
//! the flash (the first block that is NOT part of the flash memory to be
//! used), or the address of the first word after the flash array if the last
//! block of flash is to be used.
//! \param ulSize is the size of the parameter block when stored in flash;
//! this must be a power of two less than or equal to the flash erase block
//! size (typically 1024).
//!
//! This function initializes a fault-tolerant, persistent storage mechanism
//! for a parameter block for an application.  The last several erase blocks
//! of flash (as specified by \e ulStart and \e ulEnd are used for the
//! storage; more than one erase block is required in order to be
//! fault-tolerant.
//!
//! A parameter block is an array of bytes that contain the persistent
//! parameters for the application.  The only special requirement for the
//! parameter block is that the first byte is a sequence number (explained
//! in FlashPBSave()) and the second byte is a checksum used to validate the
//! correctness of the data (the checksum byte is the byte such that the sum of
//! all bytes in the parameter block is zero).
//!
//! The portion of flash for parameter block storage is split into N
//! equal-sized regions, where each region is the size of a parameter block
//! (\e ulSize).  Each region is scanned to find the most recent valid
//! parameter block.  The region that has a valid checksum and has the highest
//! sequence number (with special consideration given to wrapping back to zero)
//! is considered to be the current parameter block.
//!
//! In order to make this efficient and effective, three conditions must be
//! met.  The first is \e ulStart and \e ulEnd must be specified such that at
//! least two erase blocks of flash are dedicated to parameter block storage.
//! If not, fault tolerance can not be guaranteed since an erase of a single
//! block will leave a window where there are no valid parameter blocks in
//! flash.  The second condition is that the size (\e ulSize) of the parameter
//! block must be an integral divisor of the size of an erase block of flash.
//! If not, a parameter block will end up spanning between two erase blocks of
//! flash, making it more difficult to manage.  The final condition is that the
//! size of the flash dedicated to parameter blocks (\e ulEnd - \e ulStart)
//! divided by the parameter block size (\e ulSize) must be less than or equal
//! to 128.  If not, it will not be possible in all cases to determine which
//! parameter block is the most recent (specifically when dealing with the
//! sequence number wrapping back to zero).
//!
//! When the microcontroller is initially programmed, the flash blocks used for
//! parameter block storage are left in an erased state.
//!
//! This function must be called before any other flash parameter block
//! functions are called.
//!
//! \return None.
//
//*****************************************************************************
unsigned char *
Stel_FlashPBInit(unsigned long ulStart, unsigned long ulEnd, unsigned long ulSize)
{
    unsigned char *pucOffset, *pucCurrent;
    unsigned char ucOne, ucTwo;

    // REG: added to avoid using globals from TI driver
    unsigned char *pucFlashPBStart = (unsigned char *)ulStart;
    unsigned char *pucFlashPBEnd = (unsigned char *)ulEnd;

//	char pcBuff[10];

    //
    // Check the arguments.
    //
    ASSERT((ulStart % FLASH_ERASE_SIZE) == 0);
    ASSERT((ulEnd % FLASH_ERASE_SIZE) == 0);
    ASSERT((FLASH_ERASE_SIZE % ulSize) == 0);

//    // Initialize to NULL REG
//    pucCurrent = 0;

    //
    // Set the number of clocks per microsecond to enable the flash controller
    // to properly program the flash.
    //
    FlashUsecSet(SysCtlClockGet() / 1000000);

    //
    // Save the characteristics of the flash memory to be used for storing
    // parameter blocks.
    //
//    g_pucFlashPBStart = (unsigned char *)ulStart;
//    g_pucFlashPBEnd = (unsigned char *)ulEnd;
//    g_ulFlashPBSize = ulSize;

    //
    // Loop through the portion of flash memory used for storing parameter
    // blocks.
    //
    for(pucOffset = pucFlashPBStart, pucCurrent = 0;
        pucOffset < pucFlashPBEnd; pucOffset += ulSize)
    {
        //
        // See if this is a valid parameter block (i.e. the checksum is
        // correct).
        //
        if(Stel_FlashPBIsValid(pucOffset, ulSize))
        {
            //
            // See if a valid parameter block has been previously found.
            //
            if(pucCurrent != 0)
            {
                //
                // Get the sequence numbers for the current and new parameter
                // blocks.
                //
                ucOne = pucCurrent[0];
                ucTwo = pucOffset[0];

                //
                // See if the sequence number for the new parameter block is
                // greater than the current block.  The comparison isn't
                // straightforward since the one byte sequence number will wrap
                // after 256 parameter blocks.
                //
                if(((ucOne > ucTwo) && ((ucOne - ucTwo) < 128)) ||
                   ((ucTwo > ucOne) && ((ucTwo - ucOne) > 128)))
                {
                    //
                    // The new parameter block is older than the current
                    // parameter block, so skip the new parameter block and
                    // keep searching.
                    //
                    continue;
                }
            }

            //
            // The new parameter block is more recent than the current one, so
            // make it the new current parameter block.
            //
            pucCurrent = pucOffset;
        }
    }

//    PrintStr("\r\nFlashPBInit pucCurrent: ");
//    Stel_PrintReg((uint32_t)pucCurrent, pcBuff);

    //
    // Save the address of the most recent parameter block found.  If no valid
    // parameter blocks were found, this will be a NULL pointer.
    // REG: return the pointer for code use.
    return pucCurrent;
}


//*****************************************************************************
//
//! Writes a new parameter block to flash.
//!
//! \param pucBuffer is the address of the parameter block to be written to
//! flash.
//!
//! This function will write a parameter block to flash.  Saving the new
//! parameter blocks involves three steps:
//!
//! - Setting the sequence number such that it is one greater than the sequence
//!   number of the latest parameter block in flash.
//! - Computing the checksum of the parameter block.
//! - Writing the parameter block into the storage immediately following the
//!   latest parameter block in flash; if that storage is at the start of an
//!   erase block, that block is erased first.
//!
//! By this process, there is always a valid parameter block in flash.  If
//! power is lost while writing a new parameter block, the checksum will not
//! match and the partially written parameter block will be ignored.  This is
//! what makes this fault-tolerant.
//!
//! Another benefit of this scheme is that it provides wear leveling on the
//! flash.  Since multiple parameter blocks fit into each erase block of flash,
//! and multiple erase blocks are used for parameter block storage, it takes
//! quite a few parameter block saves before flash is re-written.
//!
//! \return None.
//
//*****************************************************************************
int
Stel_FlashPBSave(unsigned char *pucBuffer, unsigned long ulBandNo)
{
    unsigned char *pucNew;
    unsigned long ulIdx, ulSum;

    unsigned char *pucFlashPBCurrent;
    unsigned char *pucFlashPBStart;
    unsigned char *pucFlashPBEnd;
    unsigned long ulFlashPBSize;

//    char pcBuff[10];

    // REG: hard-coded values switched by function argument.
    if (ulBandNo == CAL_BAND_INDEX_UHF)
    {
    	// UHF
    	pucFlashPBCurrent = Stel_FlashPBInit(FLASH_PB_START_UHF,
    										 FLASH_PB_END_UHF,
    										 sizeof(UHFCalTable_t));
    	pucFlashPBStart = (unsigned char *)FLASH_PB_START_UHF;
    	pucFlashPBEnd = (unsigned char *)FLASH_PB_END_UHF;
    	ulFlashPBSize = sizeof(UHFCalTable_t);
    }
    else if (ulBandNo == CAL_BAND_INDEX_WIFI)
    {
    	// WiFi
    	pucFlashPBCurrent = Stel_FlashPBInit(FLASH_PB_START_WIFI,
    										 FLASH_PB_END_WIFI,
    										 sizeof(WiFiCalTable_t));
		pucFlashPBStart = (unsigned char *)FLASH_PB_START_WIFI;
		pucFlashPBEnd = (unsigned char *)FLASH_PB_END_WIFI;
		ulFlashPBSize = sizeof(WiFiCalTable_t);
    }
    else
    {
    	return 1;
    }

//    PrintStr("\r\npucFlashPBCurrent: ");
//    Stel_PrintReg((uint32_t)pucFlashPBCurrent, pcBuff);

    //
    // Check the arguments.
    //
    ASSERT(pucBuffer != (void *)0);

    //
    // See if there is a valid parameter block in flash.
    //
    if(pucFlashPBCurrent)
    {
        //
        // Set the sequence number to one greater than the most recent
        // parameter block.
        //
        pucBuffer[0] = pucFlashPBCurrent[0] + 1;

        //
        // Try to write the new parameter block immediately after the most
        // recent parameter block.
        //
        pucNew = pucFlashPBCurrent + ulFlashPBSize;
        if(pucNew == pucFlashPBEnd)
        {
            pucNew = pucFlashPBStart;
        }
    }
    else
    {
        //
        // There is not a valid parameter block in flash, so set the sequence
        // number of this parameter block to zero.
        //
        pucBuffer[0] = 0;

        //
        // Try to write the new parameter block at the beginning of the flash
        // space for parameter blocks.
        //
        pucNew = pucFlashPBStart;
    }

    //
    // Compute the checksum of the parameter block to be written.
    //
    for(ulIdx = 0, ulSum = 0; ulIdx < ulFlashPBSize; ulIdx++)
    {
        ulSum -= pucBuffer[ulIdx];
    }

    //
    // Store the checksum into the parameter block.
    //
    pucBuffer[1] += ulSum;

    //
    // Look for a location to store this parameter block.  This infinite loop
    // will be explicitly broken out of when a valid location is found.
    //
    while(1)
    {
        //
        // See if this location is at the start of an erase block.
        //
        if(((unsigned long)pucNew & 1023) == 0)
        {
            //
            // Erase this block of the flash.  This does not assume that the
            // erase succeeded in case this block of the flash has become bad
            // through too much use.  Given the extremely low frequency that
            // the parameter blocks are written, this will likely never fail.
            // But, that assumption is not made in order to be safe.
            //
            FlashErase((unsigned long)pucNew);
        }

        //
        // Loop through this portion of flash to see if is all ones (i.e. it
        // is an erased portion of flash).
        //
        for(ulIdx = 0; ulIdx < ulFlashPBSize; ulIdx++)
        {
            if(pucNew[ulIdx] != 0xff)
            {
                break;
            }
        }

        //
        // If all bytes in this portion of flash are ones, then break out of
        // the loop since this is a good location for storing the parameter
        // block.
        //
        if(ulIdx == ulFlashPBSize)
        {
            break;
        }

        //
        // Increment to the next parameter block location.
        //
        pucNew += ulFlashPBSize;
        if(pucNew == pucFlashPBEnd)
        {
            pucNew = pucFlashPBStart;
        }

        //
        // If every possible location has been checked and none are valid, then
        // it will not be possible to write this parameter block.  Simply
        // return without writing it.
        //
        if((pucFlashPBCurrent && (pucNew == pucFlashPBCurrent)) ||
           (!pucFlashPBCurrent && (pucNew == pucFlashPBStart)))
        {
            return 1;
        }
    }

//    PrintStr("\r\n pucBuffer: ");
//		Stel_PrintReg((uint32_t)pucBuffer, pcBuff);
//	PrintStr("\r\n pucNew: ");
//		Stel_PrintReg((uint32_t)pucNew, pcBuff);
//	PrintStr("\r\n ulFlashPBSize: ");
//		Stel_PrintReg((uint32_t)ulFlashPBSize, pcBuff);
//	PrintStr("\r\n UHF Size: ");
//		Stel_PrintReg((uint32_t)UHF_CAL_TABLE_SIZE, pcBuff);
//	PrintStr("\r\n WiFi Size: ");
//		Stel_PrintReg((uint32_t)WIFI_CAL_TABLE_SIZE, pcBuff);

    //
    // Write this parameter block to flash.
    //
    FlashProgram((unsigned long *)pucBuffer, (unsigned long)pucNew,
                 ulFlashPBSize);

    //
    // Compare the parameter block data to the data that should now be in
    // flash.  Return if any of the data does not compare, leaving the previous
    // parameter block in flash as the most recent (since the current parameter
    // block failed to properly program).
    //
    for(ulIdx = 0; ulIdx < ulFlashPBSize; ulIdx++)
    {
        if(pucNew[ulIdx] != pucBuffer[ulIdx])
        {
            return 1;
        }
    }

    return 0;
}

//*****************************************************************************
//
// TODO
//
//*****************************************************************************
int
Stel_ProcessCalibrationCommandString(char *pcOperand)
{
	uint8_t arglen = strlen(pcOperand);
	unsigned long ulVal;

	switch (arglen)
	{
		case 0:
		{
			// Display the whole calibration table
			Stel_PrintCalibrationTable(-1);
			break;
		}
		case 1:
		{
			// Display only calibration table ulVal
			ulVal = myDec2Long(pcOperand);
			Stel_PrintCalibrationTable((int)ulVal);
			break;
		}
		case 8:
		{
			// Load Tx or Rx LOFT
			// c 01234567
			// c BGGXXYY#
//			pCalTable = Stel_GetCalibrationTable();

			// This is a Tx LOFT value
			if (pcOperand[7] == '0')
			{
				// Check Band index.
				ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);
				if (ulVal == CAL_BAND_INDEX_UHF)
				{
					// Band 00 = UHF
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					if (ulVal >= CAL_NUM_TX_GAIN_SETTINGS)
					{
						Stel_ThrowError("Gain Index OOB");
						return 1;
					}
					g_UHFCalTable.tx_loft_cal[ulVal].i =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
					g_UHFCalTable.tx_loft_cal[ulVal].q =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
				}
				else if (ulVal == CAL_BAND_INDEX_WIFI)
				{
					// Band 01 = WiFi
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					if (ulVal >= CAL_NUM_TX_GAIN_SETTINGS)
					{
						Stel_ThrowError("Gain Index OOB");
						return 1;
					}
					g_WiFiCalTable.tx_loft_cal[ulVal].i =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
					g_WiFiCalTable.tx_loft_cal[ulVal].q =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
				}
				else
				{
					Stel_ThrowError("Bad band #");
					return 1;
				}
			}
			// This is an RX LOFT value
			else if (pcOperand[7] == '1')
			{
				// Check band index
				ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);
				if (ulVal == CAL_BAND_INDEX_UHF)
				{
					// Band 00 = UHF
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					if (ulVal >= CAL_NUM_FREQ_POINTS[CAL_BAND_INDEX_UHF])
					{
						Stel_ThrowError("Freq Index OOB for UHF");
						return 1;
					}
					g_UHFCalTable.rx_loft_cal[ulVal].i =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
					g_UHFCalTable.rx_loft_cal[ulVal].q =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
				}
				else if (ulVal == CAL_BAND_INDEX_WIFI)
				{
					// Band 01 = WiFi
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					if (ulVal >= CAL_NUM_FREQ_POINTS[CAL_BAND_INDEX_WIFI])
					{
						Stel_ThrowError("Freq Index OOB for WiFi");
						return 1;
					}
					g_WiFiCalTable.rx_loft_cal[ulVal].i =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
					g_WiFiCalTable.rx_loft_cal[ulVal].q =
							(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
				}
				else
				{
					Stel_ThrowError("Bad band #");
					return 1;
				}
			}
			else
			{
				Stel_ThrowError("Bad TxRx LOFT Specifier");
				return 1;
			}


//			ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);
//			if (ulVal == CAL_BAND_INDEX_UHF)
//			{
//				// Band 00 = UHF
//				ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//				g_UHFCalTable.rx_loft_cal[ulVal].i =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 1, 2);
//				g_UHFCalTable.rx_loft_cal[ulVal].q =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//			}
//			else if (ulVal == CAL_BAND_INDEX_WIFI)
//			{
//				// Band 01 = WiFi
//				ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//				g_WiFiCalTable.rx_loft_cal[ulVal].i =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 1, 2);
//				g_WiFiCalTable.rx_loft_cal[ulVal].q =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//			}
//			else
//			{
//				Stel_ThrowError("Bad band #");
//			}
			break;
		}
//		case 7:
//		{
//			// Load Tx LOFT
//			// c 0123456
//			// c BGGXXYY
//			pCalTable = Stel_GetCalibrationTable();
//			ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);
//			if (ulVal == CAL_BAND_INDEX_UHF)
//			{
//				// Band 00 = UHF
//				ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//				pCalTable->UHFTable.tx_loft_cal[ulVal].i =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//				pCalTable->UHFTable.tx_loft_cal[ulVal].q =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
//			}
//			else if (ulVal == CAL_BAND_INDEX_WIFI)
//			{
//				// Band 01 = WiFi
//				ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
//				pCalTable->WiFiTable.tx_loft_cal[ulVal].i =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 3, 4);
//				pCalTable->WiFiTable.tx_loft_cal[ulVal].q =
//						(uint8_t) Stel_hexSubstrToLong(pcOperand, 5, 6);
//			}
//			else
//			{
//				Stel_ThrowError("Bad band #");
//			}
//			break;
//		}
		case 28:
		{
			// Load IQ Imbalance
			//   0000000000111111111122222222
			// c 0123456789012345678901234567
			// c B##XXXXXXXXYYYYYYYYZZZZZZZZ#

			// Get the calibration band number
			ulVal = Stel_hexSubstrToLong(pcOperand, 0, 0);

			// This is a Tx IQ-compensation value
			if (pcOperand[27] == '0')
			{
				if (ulVal == CAL_BAND_INDEX_UHF)
				{
					// Band 00 = UHF
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					g_UHFCalTable.tx_iq_bb_cal[ulVal].smult =
							Stel_hexSubstrToLong(pcOperand, 3, 10);
					g_UHFCalTable.tx_iq_bb_cal[ulVal].cmult =
							Stel_hexSubstrToLong(pcOperand, 11, 18);
					g_UHFCalTable.tx_iq_bb_cal[ulVal].gmult =
							Stel_hexSubstrToLong(pcOperand, 19, 26);
				}
				else if (ulVal == CAL_BAND_INDEX_WIFI)
				{
					// Band 01 = WiFi
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					g_WiFiCalTable.tx_iq_bb_cal[ulVal].smult =
							Stel_hexSubstrToLong(pcOperand, 3, 10);
					g_WiFiCalTable.tx_iq_bb_cal[ulVal].cmult =
							Stel_hexSubstrToLong(pcOperand, 11, 18);
					g_WiFiCalTable.tx_iq_bb_cal[ulVal].gmult =
							Stel_hexSubstrToLong(pcOperand, 19, 26);
				}
				else
				{
					Stel_ThrowError("Bad band #");
				}
			}
			// This is an Rx IQ-compensation value
			else if (pcOperand[27] == '1')
			{
				if (ulVal == CAL_BAND_INDEX_UHF)
				{
					// Band 00 = UHF
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					g_UHFCalTable.rx_iq_bb_cal[ulVal].smult =
							Stel_hexSubstrToLong(pcOperand, 3, 10);
					g_UHFCalTable.rx_iq_bb_cal[ulVal].cmult =
							Stel_hexSubstrToLong(pcOperand, 11, 18);
					g_UHFCalTable.rx_iq_bb_cal[ulVal].gmult =
							Stel_hexSubstrToLong(pcOperand, 19, 26);
				}
				else if (ulVal == CAL_BAND_INDEX_WIFI)
				{
					// Band 01 = WiFi
					ulVal = Stel_hexSubstrToLong(pcOperand, 1, 2);
					g_WiFiCalTable.rx_iq_bb_cal[ulVal].smult =
							Stel_hexSubstrToLong(pcOperand, 3, 10);
					g_WiFiCalTable.rx_iq_bb_cal[ulVal].cmult =
							Stel_hexSubstrToLong(pcOperand, 11, 18);
					g_WiFiCalTable.rx_iq_bb_cal[ulVal].gmult =
							Stel_hexSubstrToLong(pcOperand, 19, 26);
				}
				else
				{
					Stel_ThrowError("Bad band #");
				}
			}
			else
			{
				Stel_ThrowError("Append Tx/Rx selector to IQ vals");
			}
			break;
		}
		default:
		{
			// The operand wasn't 0, 1, 8, or 28, so we don't know
			// how to parse it.
			Stel_ThrowError("unknown cal table operand");
			return 1;
		}
	}

	return 0;
}


//*****************************************************************************
//
// Loads stored calibration values that are frequency-dependent. This is currently
// I/Q imbalance calibration values as well as Rx LOFT calibration, but could be
// extended, if desired.
//
// \param ulTargetFreq_kHz the Tx center frequency for which to load cal values
// \param dir enum (TX or RX) signaling which direction this frequency is being set
//            for; different things are calibrated for different directions
//
// \return Zero if no errors occurred. Returns a positive value if error occurred.
//
//*****************************************************************************
int
Lime_LoadFrequencyDependentCalValues(unsigned long ulTargetFreq_kHz, Xmission_Dir_t dir)
{
	int iBand, ii, iInd, errors;
	unsigned long ulBuff;
	unsigned long ulCur;
//	CalTable_t * pCalTable;

	errors = 0;

	// We allow the disabling of automatic loading of the calibration
	// values. This is okay, but we should certainly print a warining
	if (trx.automaticCalLoadingEnabled == FALSE)
	{
		// NOTE: this is not treated as an error, else we'd fail
		//       startup of the radio all the time.
		PrintStr("Skipping Cal Load");
		__newline
		return 0;
	}

	// Try to retrieve the current calibration table
//	pCalTable = Stel_GetCalibrationTable();
//	if (pCalTable == NULL)
//	{
//		Stel_ThrowError("53");
//		return 1;
//	}
	// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Figure out which band this center frequency belongs to. Bands are defined
	// as distinct frequency ranges where RF parameters are similar and for
	// which the analog front end has been optimized for. It is a distinction
	// based on physical constraints and practical considerations, Dr. Dave.
	iBand = -1;
	for (ii = 0; ii < CAL_NUMBER_OF_BANDS; ii++)
	{
		if ( (ulTargetFreq_kHz >= CAL_LOWER_KHZ[ii] - (CAL_STEP_KHZ[ii]>>1))
		  && (ulTargetFreq_kHz <= CAL_UPPER_KHZ[ii] + (CAL_STEP_KHZ[ii]>>1)) )
		 {
			iBand = ii;
			break;
		 }
	}
	// Check if a band was found. If not, that's okay, but it means we don't have
	// calibration values stored for this center frequency.
	if (iBand == -1)
	{
		// NOTE: again, we're not treating this as an error or else bootstrapping
		//		 results in a lot of irrelevant errors.
		PrintStr("No Calib Vals for Target Freq. On startup this is OK.");
		__newline
		return 0;
	}

//	//debug
//	PrintStr("\r\nFound Band: ");
//	myLong2Hex((unsigned long)iBand, pcBuff, 2);
//	PrintStr((unsigned char *)pcBuff);
//	__newline

	// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Try to find the closest frequency to the center frequency in the
	// calibration table. We search for the frequency with the minimum distance.
	ulBuff = UL_MAX;
	iInd = -1;
	for (ii = 0; ii < CAL_NUM_FREQ_POINTS[iBand]; ii++)
	{
		ulCur = abs( ulTargetFreq_kHz - (CAL_LOWER_KHZ[iBand] + ii*CAL_STEP_KHZ[iBand]) );

//		//debug
//		PrintStr("\r\nCheck Tx Freq: ");
//		Stel_PrintReg(ulCur, pcBuff);
//		__newline

		if ( ulCur < ulBuff )
		{
			// This value is better
			iInd = ii;
			ulBuff = ulCur;
		}
	}
	if (iInd == -1)
	{
		Stel_ThrowError("57");
		return 1;
	}

//		//debug
//		PrintStr("\r\nNEAREST Tx Freq: ");
//		Stel_PrintReg((CAL_LOWER_KHZ[iBand] + iInd*CAL_STEP_KHZ[iBand]), pcBuff);
//		__newline
//		PrintStr("\r\nCURRENT Tx Freq: ");
//		Stel_PrintReg(ulTargetFreq_kHz, pcBuff);
//		__newline

	// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Because the calibration table is different for each band, we have
	// hard-coded the different band tables, thus need this switch.
	// #sosumi
	switch(iBand)
	{
		case CAL_BAND_INDEX_UHF:
			if (dir == TX)
			{
				errors += BB_SetIQCalibration(BB_IQCAL_TX_SINMULT_R,
						g_UHFCalTable.tx_iq_bb_cal[iInd].smult);
				errors += BB_SetIQCalibration(BB_IQCAL_TX_COSMULT_R,
						g_UHFCalTable.tx_iq_bb_cal[iInd].cmult);
				errors += BB_SetIQCalibration(BB_IQCAL_TX_GAINMULT_R,
						g_UHFCalTable.tx_iq_bb_cal[iInd].gmult);
			}
			else // RX
			{
				// NOTE: because of the way Naren calculates calibration register values
				//       the codes stored in the calibration table are the RAW register
				//       codes. The Lime_SetRXFE_DCO() function performs a translation
				//       from the sign-magnitude format to monotonic number line for ease
				//       of hand-calibration. Lime_SetRawRXFE_DCO() doesn't modify the
				//       codes at all.
				errors += Lime_SetRawRXFE_DCO(LMS_DCO_RXFE_I_R,
						g_UHFCalTable.rx_loft_cal[iInd].i);
				errors += Lime_SetRawRXFE_DCO(LMS_DCO_RXFE_Q_R,
						g_UHFCalTable.rx_loft_cal[iInd].q);
				// We recently added BB calibration for RX IQ-imbalance compensation because
				// it was found to be rather large.
				errors += BB_SetIQCalibration(BB_IQCAL_RX_SINMULT_R,
						g_UHFCalTable.rx_iq_bb_cal[iInd].smult);
				errors += BB_SetIQCalibration(BB_IQCAL_RX_COSMULT_R,
						g_UHFCalTable.rx_iq_bb_cal[iInd].cmult);
				errors += BB_SetIQCalibration(BB_IQCAL_RX_GAINMULT_R,
						g_UHFCalTable.rx_iq_bb_cal[iInd].gmult);
			}
			break;
		case CAL_BAND_INDEX_WIFI:
			if (dir == TX)
			{
				errors += BB_SetIQCalibration(BB_IQCAL_TX_SINMULT_R,
						g_WiFiCalTable.tx_iq_bb_cal[iInd].smult);
				errors += BB_SetIQCalibration(BB_IQCAL_TX_COSMULT_R,
						g_WiFiCalTable.tx_iq_bb_cal[iInd].cmult);
				errors += BB_SetIQCalibration(BB_IQCAL_TX_GAINMULT_R,
						g_WiFiCalTable.tx_iq_bb_cal[iInd].gmult);
			}
			else // RX
			{
				// NOTE: because of the way Naren calculates calibration register values
				//       the codes stored in the calibration table are the RAW register
				//       codes. The Lime_SetRXFE_DCO() function performs a translation
				//       from the sign-magnitude format to monotonic number line for ease
				//       of hand-calibration. Lime_SetRawRXFE_DCO() doesn't modify the
				//       codes at all.
				errors += Lime_SetRawRXFE_DCO(LMS_DCO_RXFE_I_R,
						g_WiFiCalTable.rx_loft_cal[iInd].i);
				errors += Lime_SetRawRXFE_DCO(LMS_DCO_RXFE_Q_R,
						g_WiFiCalTable.rx_loft_cal[iInd].q);
				// We recently added BB calibration for Rx IQ-imbalance compensation because
				// it was found to be rather large.
				errors += BB_SetIQCalibration(BB_IQCAL_RX_SINMULT_R,
						g_WiFiCalTable.rx_iq_bb_cal[iInd].smult);
				errors += BB_SetIQCalibration(BB_IQCAL_RX_COSMULT_R,
						g_WiFiCalTable.rx_iq_bb_cal[iInd].cmult);
				errors += BB_SetIQCalibration(BB_IQCAL_RX_GAINMULT_R,
						g_WiFiCalTable.rx_iq_bb_cal[iInd].gmult);
			}
			break;
		default:
			Stel_ThrowError("WifiCalBandLoad");
			return 1;
	}

	//Done.
	return errors;
}

//*****************************************************************************
//
// Load Tx LOFT DC compensation calibration values from calibration table based
// on the current end-to-end Tx gain setting. This is the primary reason why the
// Lime_SetMasterTxGain() function should always be used to set the gain rather
// than one of the other individual helper functions.
//
// \param ulTargetGain_dB the new gain in dB
//
// \return a zero upon success, or non-zero if any errors occur.
//
//*****************************************************************************
int
Lime_LoadGainDependentCalValues(unsigned long ulTargetGain_dB)
{
	int iBand, ii, errors;
//	CalTable_t * pCalTable;

	errors = 0;

	// We allow the disabling of automatic loading of the calibration
	// values. This is okay, but we should certainly print a warining
	if (trx.automaticCalLoadingEnabled == FALSE)
	{
		// NOTE: don't treat as an error or else bootstrapping throws irrelevant errors
		PrintStr("Skipping Calibration Load");
		__newline
		return 0;
	}

//	// Try to retrieve the current calibration table
//	pCalTable = Stel_GetCalibrationTable();
//	if (pCalTable == NULL)
//	{
//		Stel_ThrowError("53");
//		return 1;
//	}
	// Figure out which band this center frequency belongs to. Bands are defined
	// as distinct
	iBand = -1;
	for (ii = 0; ii < CAL_NUMBER_OF_BANDS; ii++)
	{
		if ( (trx.ulCurrentTxFreq >= CAL_LOWER_KHZ[ii] - (CAL_STEP_KHZ[ii]>>1))
		  && (trx.ulCurrentTxFreq <= CAL_UPPER_KHZ[ii] + (CAL_STEP_KHZ[ii]>>1)) )
		 {
			iBand = ii;
			break;
		 }
	}
	// Check if a band was found. If not, that's okay, but it means we don't have
	// calibration values stored for this center frequency.
	if (iBand == -1)
	{
		// NOTE: don't treat as an error or else bootstrapping throws irrelevant errors
		PrintStr("No TxLOFT Calib Vals for Current Freq");
		__newline
		return 0;
	}

	// Because the calibration table is different for each band, we have
	// hard-coded the different band tables, thus need this switch.
	// #sosumi
	switch(iBand)
	{
		case CAL_BAND_INDEX_UHF:
			errors += Lime_SetTXRF_DCO(LMS_DCO_TXRF_I_R,
					g_UHFCalTable.tx_loft_cal[ulTargetGain_dB].i);
			errors += Lime_SetTXRF_DCO(LMS_DCO_TXRF_Q_R,
					g_UHFCalTable.tx_loft_cal[ulTargetGain_dB].q);
			break;
		case CAL_BAND_INDEX_WIFI:
			errors += Lime_SetTXRF_DCO(LMS_DCO_TXRF_I_R,
					g_WiFiCalTable.tx_loft_cal[ulTargetGain_dB].i);
			errors += Lime_SetTXRF_DCO(LMS_DCO_TXRF_Q_R,
					g_WiFiCalTable.tx_loft_cal[ulTargetGain_dB].q);
			break;
		default:
			Stel_ThrowError("LoadGainCalValErr");
			return 1;
	}

	return errors;
}

//*****************************************************************************
//
// Accessor for doing some checks on status in the main function. Should not
// generally be used.
//
//*****************************************************************************
UHFCalTable_t *
Stel_GetUHFCalTablePointer(void)
{
	return &g_UHFCalTable;
}

//*****************************************************************************
//
// Accessor for doing some checks on status in the main function. Should not
// generally be used.
//
//*****************************************************************************
WiFiCalTable_t *
Stel_GetWiFiCalTablePointer(void)
{
	return &g_WiFiCalTable;
}

//*****************************************************************************
//
//! Determines if the parameter block at the given address is valid.
//!
//! \param pucOffset is the address of the parameter block to check.
//!
//! This function will compute the checksum of a parameter block in flash to
//! determine if it is valid.
//!
//! \return Returns one if the parameter block is valid and zero if it is not.
//
//*****************************************************************************
static unsigned long
Stel_FlashPBIsValid(unsigned char *pucOffset, unsigned long ulFlashPBSize)
{
    unsigned long ulIdx, ulSum;

    //
    // Check the arguments.
    //
    ASSERT(pucOffset != (void *)0);

    //
    // Loop through the bytes in the block, computing the checksum.
    //
    for(ulIdx = 0, ulSum = 0; ulIdx < ulFlashPBSize; ulIdx++)
    {
        ulSum += pucOffset[ulIdx];
    }

    //
    // The checksum should be zero, so return a failure if it is not.
    //
    if((ulSum & 255) != 0)
    {
        return(0);
    }

    //
    // If the sum is equal to the size * 255, then the block is all ones and
    // should not be considered valid.
    //
    if((ulFlashPBSize * 255) == ulSum)
    {
        return(0);
    }

    //
    // This is a valid parameter block.
    //
    return(1);
}

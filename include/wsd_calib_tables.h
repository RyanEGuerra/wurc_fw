/*
 * calib_tables.h
 *
 *  Created on: Feb 22, 2014
 *      Author: rng
 */

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "inc/lm3s5r36.h"

#include "include/wsd_defines.h"
#include "include/bb_interface_lib.h"
#include "include/wsd_io.h"
#include "include/wsd_lms6002d_lib.h"


#ifndef CALIB_TABLES_H_
#define CALIB_TABLES_H_

#ifndef NULL
#define NULL          0
#endif

// =============================================================================
// DCO & LOFT Calibration Values Parameters ====================================

// Version 1.10 and greater: each calibration band has CAL_LOFT_SIZE LOFT
// calibration entries starting at CAL_LOWER_KHZ and ending at
// CAL_UPPER_KHZ. These values increase in frequency in steps of
// CAL_STEP_KHZ and are identified by the following indices:
// * CAL_BAND_INDEX_UHF - UHF table index
//	 Note: though we can't use 700 MHz in the US, these values are included here
//   since it's better to include them now than to have to go back and rewrite
//   to support Europe.
//   http://en.wikipedia.org/wiki/Television_channel_frequencies
// * CAL_BAND_INDEX_WIFI - WiFi table index
// 	 Note: the WiFi table includes a phantom channel between 13 and 14 that
//   doesn't exist, but including it make the math easier.
//	 http://en.wikipedia.org/wiki/List_of_WLAN_channels
//
//FIXME
#define CAL_NUMBER_OF_BANDS			2
#define CAL_NUM_TX_GAIN_SETTINGS	26
#define CAL_BAND_INDEX_UHF			0
#define CAL_BAND_INDEX_WIFI			1
static const unsigned long CAL_NUM_FREQ_POINTS[] = {26, 		16};
static const unsigned long CAL_LOWER_KHZ[]  	 = {473000, 	2412000};
static const unsigned long CAL_UPPER_KHZ[]  	 = {773000, 	2487000};
static const unsigned long CAL_STEP_KHZ[] 		 = {12000, 	5000};

// Instead of relying on hardware to calculate sine/cosine, we pre-compute
// all three BB multiplier values and store in the calibration table.
// We've found that these values are Tx frequency-dependent, so each of these
// calibration 3-tuples is specific for a given Tx center frequency.
typedef struct {
	uint32_t smult;
	uint32_t cmult;
	uint32_t gmult;
} IQCalEntry_t;							// 12 Bytes

// Tx LOFT DC compensation blocks have been discovered to be dependent on the
// Tx gain setting. Each of these Tx LOFT calibration tuples is gain-dependent
// and tuned for a gain setting between [0, 61] selected thus:
// TxVGA1 = [0, 25], TxVGA2 = [0, 25], TxVGA1 = [26, 31]
typedef struct {
	uint8_t i;
	uint8_t q;
} LOFTCalEntry_t;						// 2 Bytes


// "Parameter Block" structures used to define the factory calibration
// values of the transceiver, for both UHF and WiFi.
//
// Note: From the pb_api documentation:
//! A parameter block is an array of bytes that contain the persistent
//! parameters for the application.  The only special requirement for the
//! parameter block is that the first byte is a sequence number (explained
//! in FlashPBSave()) and the second byte is a checksum used to validate the
//! correctness of the data (the checksum byte is the byte such that the sum of
//! all bytes in the parameter block is zero).

// We word-align the UHF table and also make it fit cleanly into Flash
// pages by padding each table with additional bytes to make it 1024 Bytes
// long.
typedef struct {
	uint8_t 		ucSeqNo;								// 1 Byte
	uint8_t 		ucCheckSum;								// 1 Byte
	uint8_t 		padding_1[2];							// 2 Bytes
	LOFTCalEntry_t 	rx_loft_cal[26];						// 26*2 = 52 Bytes
	LOFTCalEntry_t 	tx_loft_cal[CAL_NUM_TX_GAIN_SETTINGS];	// 26*2 = 52 Bytes
	IQCalEntry_t	tx_iq_bb_cal[26];						// 26*4*3 = 312 Bytes
	IQCalEntry_t	rx_iq_bb_cal[26];						// 26*4*3 = 312 Bytes
	uint8_t 		padding_2[1024-732];					// 292
} UHFCalTable_t; 											// 732 Bytes + padding

#define UHF_CAL_TABLE_SIZE 512

// The WiFi table fits into half of a flash page.
typedef struct {
	uint8_t 		ucSeqNo;								// 1 Byte
	uint8_t 		ucCheckSum;								// 1 Byte
	uint8_t 		padding_1[2];							// 2 Bytes
	LOFTCalEntry_t 	rx_loft_cal[16];						// 16*2 = 32 Bytes
	LOFTCalEntry_t 	tx_loft_cal[CAL_NUM_TX_GAIN_SETTINGS];	// 26*2 = 52 Bytes
	IQCalEntry_t	tx_iq_bb_cal[16];						// 16*4*3 = 192 Bytes
	IQCalEntry_t	rx_iq_bb_cal[16];						// 16*4*3 = 192 Bytes
	uint8_t 		padding_2[512-472];						// 40 Bytes
} WiFiCalTable_t; 											// 472 Bytes + padding

#define WIFI_CAL_TABLE_SIZE 1024

//typedef struct {
//	uint8_t ucSeqNo;			// 1 Byte
//	uint8_t ucCheckSum;			// 1 Byte
//	uint8_t padding_1[2];		// 2 Bytes
//	UHFCalTable_t 	UHFTable;	// 680 Bytes
//	WiFiCalTable_t	WiFiTable;	// 248 Bytes
//	uint8_t padding_2[1024-932];	// Total used size = 1024 Bytes
//} CalTable_t;

// Defines where parameter flash memory starts and stops for the purpose of
// initializing the "ring buffer" parameter block driver. The linker command
// file must stop flash memory at FLASH_PB_START or else run the risk of mixing
// execution code with parameter block values. These need to be on flash page
// boundaries (Stellaris page size = 1024 Bytes = 0x400)
// Note: the end value points to the block AFTER the last valid block.
//#define FLASH_PB_START			0x3F400	//0x40000 - C00
//#define FLASH_PB_END			0x3FC00 //0x40000 - 400
#define FLASH_PB_START_UHF		0x3E000
#define FLASH_PB_END_UHF		0x3F000

#define FLASH_PB_START_WIFI		0x3F400
#define FLASH_PB_END_WIFI		0x3FC00 //0x40000 - 400

// =============================================================================


#endif /* CALIB_TABLES_H_ */



int Stel_InitFlashCalibrationTables(void);
UHFCalTable_t *  Stel_GetUHFCalibrationTable(void);
WiFiCalTable_t * Stel_GetWiFiCalibrationTable(void);
int Stel_AtomicCommitCalibrationTable();
void Stel_PrintCalibrationTable(int band_no);
int Stel_ProcessCalibrationCommandString(char *pcOperand);

int Lime_LoadFrequencyDependentCalValues(unsigned long ulTargetFreq_kHz, Xmission_Dir_t dir);
int Lime_LoadGainDependentCalValues(unsigned long ulTargetGain_dB);

UHFCalTable_t * Stel_GetUHFCalTablePointer(void);
WiFiCalTable_t * Stel_GetWiFiCalTablePointer(void);

// Internal functions not meant for external calls
unsigned char * Stel_FlashPBInit(unsigned long ulStart, unsigned long ulEnd, unsigned long ulSize);
int Stel_SetUHFCalibrationTable(UHFCalTable_t * psNewTable);
int Stel_SetWiFiCalibrationTable(WiFiCalTable_t * psNewTable);
int Stel_FlashPBSave(unsigned char *pucBuffer, unsigned long ulBandNo);
static unsigned long Stel_FlashPBIsValid(unsigned char *pucOffset, unsigned long ulFlashPBSize);

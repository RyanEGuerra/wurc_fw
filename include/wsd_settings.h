/*
 * wsd_settings.h
 * 	Global settings for the WSB C project.
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 *   CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Created: April 23, 2013
 * Edited: August 16, 2013
 * Authors:	Ryan E. Guerra (me@ryaneguerra.com)
 */

// From revision 1.03 onward, both of these compiler flags should be set at
// all times.
#define __ENABLE_UART__
#define __ENABLE_USB__
//#define __DEBUG_UART__

// Flags to indicate the version of hardware this firmware is compiled for.
// Aside from a number of architectural changes, this should be able to
// cleanly allow compilation of firmware for the various versions.
// Wideband UHF Radio Card (WURC) Revision Flag
// __VOLO_WURC_REV_BLUE__ 	- Revision 2, Blue Board (requires air wire for 2.5V IO, internal LNA only)
// __VOLO_WURC_REV_GREEN__ 	- Revision 3, Green Board (fixed pwr, internal LNA only)
// __VOLO_WURC_REV_RED__ 	- Revision 4, Red Board (external LNA, internal deactivated)
#define __VOLO_WURC_REV_RED__

#ifndef __WSD_SETTING_H__
#define __WSD_SETTING_H__

#include "wsd_gpio.h"

// =============================================================================
// Firmware Version ============================================================
static const unsigned char* pucWSDFirmVersion = "2.40";
static const unsigned char* pucWSDFirmRevDate = __DATE__;
static const unsigned char* pucWSDFirmRevTime = __TIME__;
// =============================================================================

// Startup Default Gain Levels
// These are all set when Lime_PwrOnInitializationSequence() is called
#define LMS_STARTUP_TXMASTER_GAIN	0
#define LMS_STARTUP_RXVGA1_GAIN		115
#define LMS_STARTUP_RXVGA2_GAIN		0

// On the red revision boards, we use the external LNA due to it's superior
// NF and gain. On the Blue and Green revision boards, we use the internal
// LNA. A setting of 0 is a bypass setting.
#ifdef __VOLO_WURC_REV_RED__
	#define LMS_STARTUP_LNA_GAIN		0
#endif //__VOLO_WURC_REV_RED__
#if defined(__VOLO_WURC_REV_GREEN__) || defined(__VOLO_WURC_REV_BLUE__)
	#define LMS_STARTUP_LNA_GAIN		2
#endif //defined(__VOLO_WURC_REV_GREEN__) || defined(__VOLO_WURC_REV_BLUE__)


// Startup frequency settings for Tx and Rx PLL
// Note: 593 has noise on the channel that we do NOT want to deal with.
// 490 is relatively clean.
#define LMS_STARTUP_TXFREQ			490000
#define LMS_STARTUP_RXFREQ			490000

// Startup LPF bandwidth settings. These are SSB settings, so the channel BW is
// 2x this. All constants are "LMS_LPF_BW_I_F" where I is integer value and F
// is the fractional value.
#define LMS_STARTUP_TXBW			LMS_LPF_BW_6
#define LMS_STARTUP_TXBW	  		LMS_LPF_BW_6

// Yep. On the Stellaris architecture, an unsigned long is a uint32.
typedef unsigned long uint32_t;
typedef unsigned char uint8_t;
typedef unsigned char bool;

// The maximum size that an op code is allowed to take; changing this
// will break some things, so leave it be.
#define OPCODE_MAX_SIZE 1

// The maximum size that the operand following an op code is allowed to
// take. This can be changed at will, but hasn't been tested.
#define OPERAND_MAX_SIZE 20

// The maximum number of time slots to wait for the DCO_CLBR
// procedure to end.
// This consistently seems to finish in about 4 cycles. I believe this is
// actually deterministic based on the FAQ question 4.5 that claims:
// DC Calibration takes 64 clock cycles per stage.
// DC calibration clocks are derived from the PLLCLK:
//   For RXVGA2: DCCALCLK = PLLCLK/16, 5 stages
//	 For LPF: DCCALCLK = PLLCLK/256, 2 stages each for TX and RX
#define MAXIMUM_DCO_CLBR_TRY_COUNT	6

// This provides a timeout for the PrintStrUSB() function so that
// the Stellaris doesn't get caught in an infinite loop when there is no
// host plugged into the micro-USB port. We experimented and found that too-
// large a delay timed out too slowly, but too short a delay and suddenly
// arbitrary characters here and there would be dropped from the UART strings
// 5000 was about right, but I bumped it up to 9000 just to be safe.
#define USB_TIMEOUT_MAX_COUNT	9000

// How fast should the Stellaris-controlled indicator LED blink?
#define WSD_LED_BLINK_SPEED 		1000000

// The system clock speed. Saves system calls to SysCtlClockGet (80 MHz)
// These two numbers should be kept in sync
#define WSD_SYS_CLK_RATE 			80000000
#define WSD_SYS_CLK_FREQ_MHz		WSD_SYS_CLK_RATE/1000000

// The slow interface speed for the SSI0 interface (10 MHz)
#define WSD_SLOW_SSI_RATE 			10000000

// The fast interface speed for the SSI0 interface (40 MHz)
#define WSD_FAST_SSI_RATE 			40000000

// Give the speed of the PLLCLK fed as reference to the LMS6002D.
// This should be output by the FPGA's internal PLLs and should
// normally be 40 MHz.
// NOTE: this is in units of kHz!!
#define LMS_PLL_REF_CLK_RATE 		40000

// The number of idle cycles required to wait after turning on the VCO
// comparators and reading the comparator output values.
// This was inspired by the fact that when debug stepping through code,
// errors in the VCO comparator outputs were not repeatable, which indicates
// that a delay is required to allow some sort of settling. The known-bad
// case occurred at 555000 MHz with board #3.
// TODO: this value was ballparked based on some bad math, then tuned to work
//       it'd be nice to actually know how long this is.
// NOTE: 10/06/2013 - this was too slow for some boards @ 258. 300 worked, but
//       I'm bumping this constant up to 400 to be safe. We really do need to
//       figure out how long this needs to be.
#define LMS_VCO_SETTLING_CYCLES		500

// The total number of idle cycles required to wait before
// checking if the LMS internal DC calibration has finished.
//TODO - we don't know a good value for this, or how sensitive it is
#define DCO_CLBR_WAIT_CYCLES 		100

// The delay loop uses an assembly NOP to keep from compiler from optimizing
// the loop away (it'll actually do that! smart bastard)
#define __NOP_DELAY_LOOP(X) \
	do { \
		unsigned long loop_count; \
		for (loop_count = 0; loop_count < X; loop_count++) \
		{ \
			__asm("    NOP\n"); \
		} \
	} while (0)

// Loopback frequency setting in kHz
// Right now, this should be somewhere in the middle of the transmit
#define LMS_UHF_LOOPBACK_FREQ		601000		// 601 = sqrt(470*770)
#define LMS_WIFI_LOOPBACK_FREQ		2449000		// 2449 = sqrt(2400*2500)

// Startup Input/Output analog RF chain settings
// Options:
// 	LMS_LNA_DISABLED, LMS_LNA_1, LMS_LNA_2, LMS_LNA_3
// 	LMS_TXVGA2_PA1OFF_PA2OFF, LMS_TXVGA2_PA1ON_PA2OFF, LMS_TXVGA2_PA1OFF_PA2ON
#define LMS_STARTUP_TX_VGA2			LMS_TXVGA2_PA1ON_PA2OFF

//// =============================================================================
//// DCO & LOFT Calibration Values Parameters ====================================
//
//// Version 1.10 and greater: each calibration band has CAL_LOFT_SIZE LOFT
//// calibration entries starting at CAL_LOWER_KHZ and ending at
//// CAL_UPPER_KHZ. These values increase in frequency in steps of
//// CAL_STEP_KHZ and are identified by the following indices:
//// * CAL_BAND_INDEX_UHF - UHF table index
////	 Note: though we can't use 700 MHz in the US, these values are included here
////   since it's better to include them now than to have to go back and rewrite
////   to support Europe.
////   http://en.wikipedia.org/wiki/Television_channel_frequencies
//// * CAL_BAND_INDEX_WIFI - WiFi table index
//// 	 Note: the WiFi table includes a phantom channel between 13 and 14 that
////   doesn't exist, but including it make the math easier.
////	 http://en.wikipedia.org/wiki/List_of_WLAN_channels
////
////FIXME
//#define CAL_NUMBER_OF_BANDS			2
//#define CAL_NUM_TX_GAIN_SETTINGS	26
//#define CAL_BAND_INDEX_UHF			0
//#define CAL_BAND_INDEX_WIFI			1
//static const unsigned long CAL_NUM_FREQ_POINTS[] = {26, 		16};
//static const unsigned long CAL_LOWER_KHZ[]  	 = {473000, 	2412000};
//static const unsigned long CAL_UPPER_KHZ[]  	 = {773000, 	2487000};
//static const unsigned long CAL_STEP_KHZ[] 		 = {12000, 	5000};
//
//// Instead of relying on hardware to calculate sine/cosine, we pre-compute
//// all three BB multiplier values and store in the calibration table.
//// We've found that these values are Tx frequency-dependent, so each of these
//// calibration 3-tuples is specific for a given Tx center frequency.
//typedef struct {
//	uint32_t smult;
//	uint32_t cmult;
//	uint32_t gmult;
//} IQCalEntry_t;							// 12 Bytes
//
//// Tx LOFT DC compensation blocks have been discovered to be dependent on the
//// Tx gain setting. Each of these Tx LOFT calibration tuples is gain-dependent
//// and tuned for a gain setting between [0, 61] selected thus:
//// TxVGA1 = [0, 25], TxVGA2 = [0, 25], TxVGA1 = [26, 31]
//typedef struct {
//	uint8_t i;
//	uint8_t q;
//} LOFTCalEntry_t;						// 2 Bytes
//
//typedef struct {
//	LOFTCalEntry_t 	rx_loft_cal;		// 4 Bytes
//	LOFTCalEntry_t 	tx_loft_cal[26];	// 26*2 = 52 Bytes
//	IQCalEntry_t	tx_iq_bb_cal[26];	// 26*4*3 = 312 Bytes
//	IQCalEntry_t	rx_iq_bb_cal[26];	// 26*4*3 = 312 Bytes
//} UHFCalTable_t; 						// 680 Bytes
//
//typedef struct {
//	LOFTCalEntry_t 	rx_loft_cal;		// 4 Bytes
//	LOFTCalEntry_t 	tx_loft_cal[26];	// 26*2 = 52 Bytes
//	IQCalEntry_t	tx_iq_bb_cal[8];	// 8*4*3 = 96 Bytes
//	IQCalEntry_t	rx_iq_bb_cal[8];	// 8*4*3 = 96 Bytes
//} WiFiCalTable_t; 						// 248 Bytes
//
//// "Parameter Block" structure used to define the factory calibration
//// values of the transceiver.
//// Note: From the pb_api documentation:
////! A parameter block is an array of bytes that contain the persistent
////! parameters for the application.  The only special requirement for the
////! parameter block is that the first byte is a sequence number (explained
////! in FlashPBSave()) and the second byte is a checksum used to validate the
////! correctness of the data (the checksum byte is the byte such that the sum of
////! all bytes in the parameter block is zero).
//// We word-align this and also make it fit cleanly into Flash pages by padding
//// each table with additional bytes to make it 1024 Bytes long
//typedef struct {
//	uint8_t ucSeqNo;			// 1 Byte
//	uint8_t ucCheckSum;			// 1 Byte
//	uint8_t padding_1[2];		// 2 Bytes
//	UHFCalTable_t 	UHFTable;	// 680 Bytes
//	WiFiCalTable_t	WiFiTable;	// 248 Bytes
//	uint8_t padding_2[1024-932];	// Total used size = 1024 Bytes
//} CalTable_t;
//
//// Defines where parameter flash memory starts and stops for the purpose of
//// initializing the "ring buffer" parameter block driver. The linker command
//// file must stop flash memory at FLASH_PB_START or else run the risk of mixing
//// execution code with parameter block values. These need to be on flash page
//// boundaries (Stellaris page size = 1024 Bytes = 0x400)
//// Note: the end value points to the block AFTER the last valid block.
//#define FLASH_PB_START			0x3F400	//0x40000 - C00
//#define FLASH_PB_END			0x3FC00 //0x40000 - 400
//// =============================================================================

//// =============================================================================
//// Macro for inserting a new line on either interface. We have this just in case
//// we want to change the newline character.
//// ...and it's a good thing we did! Because now integrating two terminals just
//// because a lot easier, so this needs to be updated. REG - Aug 2013
//#define __newline PrintStr("\n\r");
//// =============================================================================


#endif /* __WSD_SETTING_H__ */

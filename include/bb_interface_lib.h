/*
 * bb_interface_lib.h
 *
 * Contains function prototypes and register definitions to interact with
 * a host baseband board. This is written for WARP right now, but it could
 * be adapted to other basebands in the future.
 *
 * Note: this is not perfect; not all BB interface functions have been
 * moved to this file, but the intent is to start segmenting the project
 * such that this eventually happens. This should help with maintainability
 * and scalability.
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 *   CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  Created on: Jul 28, 2013
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 */
#define __TARGET_BB_IS_WARPv3__

#ifndef BB_INTERFACE_LIB_H_
#define BB_INTERFACE_LIB_H_

// ===========================================================================
// WARP Slave Register Definitions ===========================================
#define WARP_IQCAL_TX_SINMULT_R		0x01
#define WARP_IQCAL_TX_COSMULT_R		0x02
#define WARP_IQCAL_TX_GAINMULT_R	0x03

#define WARP_ADCINPUT_ALLZEROS		0x04
#define WARP_ADCINPUT_NORMAL		0x05

#define WARP_IQCAL_RX_SINMULT_R		0x06
#define WARP_IQCAL_RX_COSMULT_R		0x07
#define WARP_IQCAL_RX_GAINMULT_R	0x08

#define WARP_SERIAL_CODE_R			0x09

// ===========================================================================
// BB-agnostic Register Definitions ==========================================
#ifdef __TARGET_BB_IS_WARPv3__
#define BB_IQCAL_TX_SINMULT_R 		WARP_IQCAL_TX_SINMULT_R
#define BB_IQCAL_TX_COSMULT_R 		WARP_IQCAL_TX_COSMULT_R
#define BB_IQCAL_TX_GAINMULT_R		WARP_IQCAL_TX_GAINMULT_R
#define BB_IQCAL_RX_SINMULT_R 		WARP_IQCAL_RX_SINMULT_R
#define BB_IQCAL_RX_COSMULT_R 		WARP_IQCAL_RX_COSMULT_R
#define BB_IQCAL_RX_GAINMULT_R		WARP_IQCAL_RX_GAINMULT_R
#define BB_SERIAL_CODE_R			WARP_SERIAL_CODE_R
#endif //__TARGET_BB_IS_WARPv3__

typedef enum {
	ALLZERO = 00,
	NORMAL = 01
} DAC_Input_t;

// ===========================================================================
// Function Prototypes =======================================================
int BB_TestBBConnection(void);
int BB_UARTSendLongWord(unsigned long ulAddr,
						unsigned long ulLongData);
int BB_SetIQCalibration(unsigned long ulBaseAddr,
						  unsigned long ulIQCalValue);
int BB_SetDACInputState(DAC_Input_t state);
int BB_UARTSendCommand(unsigned long ulCmdWord);
int BB_SetHostSerialNumber(unsigned long ulSerialNumber);


#endif /* BB_INTERFACE_LIB_H_ */

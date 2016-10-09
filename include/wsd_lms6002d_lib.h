/*
 * wsd_ssi_master.h
 * 	Contains all prototypes functions for 4-pin serial interface. This code acts as master
 * 	to a connected slave Lime Microsystems LMS6002D.
 *
 *  Created on: Feb 28, 2013
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 *      Author: Narendra Anand (nanand@rice.edu)
 *      Thanks: Holly Liang
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 *   CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  Initial calibration algorithm work is thanks to Narendra, and a big thanks to Holly for
 *  initial testing with the Lime Dev Board.
 */

#ifndef __WSD_LMS6002D_LIB_H__
#define __WSD_LMS6002D_LIB_H__

//FIXME
//#define __DEMO_CODE__

#include "stdlib.h"

#include "inc/hw_types.h"
#include "driverlib/flash.h"
#include "driverlib/rom.h"
#include "wsd_defines.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "wsd_main.h"
#include "wsd_gpio.h"
#include "wsd_io.h"
#include "bb_interface_lib.h"

/********************** TypeDefs ****************************/
typedef enum {
	RX_NORMAL = 0,
	RX_BYPASS_INT_LNA = 1,
	RX_LOOPBACK_RF = 2,
	RX_LOOPBACK_BB = 3
} TRXState_t;

// Keeps track of global variables and settings on the radio.
// This is immensely important to operations.
typedef struct {									// Default Values First
	unsigned long 	ulCurrentTxFreq;				// 0
	unsigned long 	ulCurrentRxFreq;				// 0
	unsigned long 	ulCurrentTxGain;				// 0
	unsigned long 	ulCurrentRx_I_DCO;				// 0
	unsigned long 	ulCurrentRx_Q_DCO;				// 0
	unsigned long 	ulRxFREQSEL;					// 0
	unsigned long 	ulTxRxPAOffTimeout;				// 1
	unsigned long 	ulRxTxPLLSettlingTimeout;		// 1
	TxRxSWMode_t 	TxRxSwitchingMode;				// TxRxSWMode_FDD, TxRxSWMode_TDD
	AutoCalMode_t 	CalibrationMode;				// LMS_CAL_NORMAL, LMS_CAL_RF_LOOPBACK, LMS_CAL_BB_LOOPBACK
	RFInputSrc_t	RxInputMode;					// EXTERNAL_LNA, INTERNAL_LNA
	bool			verboseOutputEnabled;			// TRUE
	bool			automaticCalLoadingEnabled;		// TRUE
	bool			automaticTxRxSwitchingEnabled; 	// FALSE // NOTE: this is set up right now for Tx/Rx in half-duplex.
	bool 			isTransmittingInUHFBand;		// TRUE  // NOTE: this is set up right now for Tx/Rx in half-duplex.
	bool			isReceivingInUHFBand;			// TRUE  // NOTE: this is set up right now for Tx/Rx in half-duplex.
	bool			flashCommitHasBeenVerified;		// FALSE
	bool			directCTRL_LNAIsActive;			// TRUE
	bool			txFSync;			// Fsync polarity, frame start
	bool			rxFSync;			//
	bool			txIQInterleave;
	bool			rxIQInterleave;
	bool			adcPhaseSel;		// 1 = falling, 0 = rising
	bool			dacClkEdgePolarity;	// 1 = negative, 0 = positive
	char			clkNonOverlapAdj;	// 11 = +300ps, 10 = +150ps, 01 = +450ps, 00 = nominal
	// RX_NORMAL, RX_BYPASS_INT_LNA, RX_LOOPBACK_RF, RX_LOOPBACK_BB
	TRXState_t 		TRXState;
} RadioState_t;

/********************** Variables ****************************/

extern unsigned long g_ulDevNull;
//extern char g_cTxBandIsUHF;

// Used for debug state tracking. Total hack.
extern char g_cTxRxSwitchState;

// All radio states, such as RF params and loopback/input settings
// are stored here so that various functions perform the proper
// operations based on operation mode.
extern RadioState_t trx;

// Keeps track of whether or not baseband-controlled Tx/Rx switching is enabled.
extern char TxRxSwitchingControlIsActive;

/********************** Prototypes ****************************/

void Stel_SetSSI0ClkSlow(void);
void Stel_SetSSI0ClkFast(void);
void Lime_InitRadioState(void);
void Lime_FastWrite(const unsigned long ulADDR, unsigned long ulDATA);
void Lime_Read(unsigned long ulADDR, unsigned long* pulDATA);
int Lime_RobustWrite(const unsigned long ulADDR, const unsigned long ulDATA);
int Lime_RobustWriteTest(const unsigned long ulAddr, const unsigned long ulData);
int Lime_ConnectionLost(void);
int Lime_GetChipInformation(void);
int Lime_GeneralDCCalibration(const unsigned long ulBase, const unsigned long ulDCAddr);
int Lime_GeneralDCCalibrationSubroutine(const unsigned long ulBase, const unsigned long ulDCAddr);
int Lime_LPFCoreDCOffsetCalibration(void);
int Lime_TXRXLPFDCOffsetCalibration(unsigned long ulBase, unsigned long ulCLKENMask);
int Lime_RXVGA2DCOffsetCalibration(void);
int Lime_LPFBandwidthTuning(void);
int Lime_NewLPFBandwidthTuning(unsigned long ulLPFBandwidthCode);
int Lime_AutoCalibrationSequence(void);
int Lime_VCOCAPCalibration(unsigned long ulPLLBaseAddr);
int Lime_SetFrequency(unsigned long ulBaseAddr, unsigned long ulTargetFrequencyKHz);
int Lime_SetFilterBandwidth(const unsigned long ulBaseAddr, const unsigned long ulBandwidthSetting);
int Lime_SetSoftEnable(unsigned long ulBitMask);
int Lime_ToggleSTXEN(void);
int Lime_ToggleSRXEN(void);
int Lime_SetClockCtrl(const unsigned long ulClkMask, const unsigned long ulOperation);

int Lime_SetTXVGA1Gain(const unsigned long ulVal);
int Lime_SetTXVGA2Gain(const unsigned long ulVal);
int Lime_SetRXVGA2Gain(const unsigned long ulVal);
int Lime_SetRXVGA1Gain(const unsigned long ulVal);
int Lime_SetMasterTxGain(const unsigned long ulNewGain);

int Lime_SelectTXVGA2_PA(const unsigned long ulMask);
int Lime_SelectRXLNA(const unsigned long ulMask);
//int Lime_LPFBypassSet(const unsigned long ulBaseAddr, const unsigned long ulMask);
int Lime_SelectActiveLNA(unsigned long ulSetting);

int Lime_SelectRFLoopback(unsigned long ulSetting);
int Lime_SelectBBLoopback(unsigned long ulSetting);

int Lime_SetDACSettings(unsigned long ulMask);
int Lime_SetADCSettings(unsigned long ulMask);
int Lime_SetDACEnable(En_Dis_t ulState);
int Lime_SetDefaultSettings(void);
int Lime_SetRegisterField(unsigned long ulRegister,
						  unsigned long ulVal,
						  unsigned long ulFieldMask);
void Lime_SetRegisterField_Fast(const unsigned long ulRegister,
							    unsigned long ulVal,
							    const unsigned long ulFieldMask);
int Lime_GetRegisterField(unsigned long ulRegister,
					  	  unsigned long ulFieldMask,
					  	  unsigned long* pulBuffer);
int Lime_ToggleRXOutSwitch(void);
int Lime_OpenRXOutSwitch(void);
int Lime_CloseRXOutSwitch(void);
int Lime_SetLNAGain(unsigned long ulVal);
int Lime_SoftReset_PAsOff(void);

int Lime_SetRXFE_DCO(unsigned long ulBaseAddr, unsigned long ulDCOVal);
int Lime_SetTXRF_DCO(unsigned long ulBaseAddr, unsigned long ulDCOVal);
unsigned long Lime_GetRXFE_DCO(unsigned long ulBaseAddr);
unsigned long Lime_GetTXRF_DCO(unsigned long ulBaseAddr);
int Lime_SetRawRXFE_DCO(unsigned long ulBaseAddr, unsigned long ulDCOVal);

int Lime_ClearTXFsync(void);
int Lime_SetTXFsync(void);
int Lime_SetRXFsync(void);
int Lime_ClearRXFsync(void);
int Lime_SetTXIQInterleave(void);
int Lime_ClearTXIQInterleave(void);
int Lime_SetRXIQInterleave(void);
int Lime_ClearRXIQInterleave(void);
int Lime_TX_SwapIQPolarity(void);
int Lime_RX_SwapIQPolarity(void);
int Lime_TX_SwapIQInterleave(void);
int Lime_RX_SwapIQInterleave(void);
int Lime_FlipDACClkEdgePolarity(void);
int Lime_FlipADCSampPhaseSel(void);
int Lime_SetClkNonOverlapPhaseAdj(unsigned long new_index);

int Lime_SelectTXVGA2(unsigned long ulArg);
int Lime_SetRXFETestModeRegisterAccess(unsigned long ulSetting);
int Lime_DirectDisableInternalLNAs();
int Lime_DirectEnableInternalLNAs();
int Lime_StartIQCalibrationMode(void);
int Lime_XceiverPwrOnInitializationSequence(void);
int Lime_SoftTXDisable(void);
int Lime_SoftTXEnable(void);
int Lime_SoftRXDisable(void);
int Lime_SoftRXEnable(void);
void Lime_DisplayRegisterFieldVal(unsigned long ulRegister,
								  unsigned long ulMask);
unsigned long Lime_GetRegisterFieldVal(unsigned long ulRegister,
									   unsigned long ulMask);
int Lime_SetChannel(unsigned long ulChanType,
					unsigned long ulChan,
					unsigned long ulChanBW_MHz);

int Lime_LoadFrequencyDependentCalValues(unsigned long ulTarget_Frequency_kHz,
								         Xmission_Dir_t dir);
int Lime_LoadGainDependentCalValues(unsigned long ulTargetGain_dB);
extern int BB_SetIQCalibration(unsigned long ulBaseAddr,
							     unsigned long ulIQCalValue);

int WSD_SetSerialNumber(unsigned long ulSerial);
int WSD_CommitSerialNumber(void);
unsigned long WSD_GetSerialNumber(void);
void WSD_PrintCurrentLOFT_DCOSettings(void);
void WSD_EnableBBTxRxSwitchingCtrl(void);
void WSD_DisableBBTxRxSwitchingCtrl(void);
int Lime_SetAUXPAState(AUXPAState_t state);

int Lime_Set_IN1SEL_MIX_RXFE(IN1SELState_t mode);
int Lime_Set_RINEN_MIX_RXFE(RINEN_MIX_RXFE_t mode);
int Lime_SetRxInputPath(RFInputSrc_t mode);
int Lime_Rx_IsWiFiBand(void);
int	Lime_Rx_IsUHFBand(void);
int Lime_Tx_IsWiFiBand(void);
int Lime_Tx_IsUHFBand(void);

int Lime_StartRFLoopbackMode(void);
int Lime_StartBBLoopbackMode(void);

int Lime_FuckingNukeEverything(void);

// I pulled this out to an external file because it was getting large.
#include "txrx_switching_macros.h"

#endif //__WSD_LMS6002D_LIB_H__

/*
 * wsd_defines.h
 *   Global defines used in the wsd_usb_client project
 *
 *   This software is licensed to Rice University where applicable, Texas Instruments
 *   where applicable and should always carry this header description. No warranty is
 *   provided with this code, nor promise of suitability for any given purpose. Do not
 *   blame us if it breaks something.
 *
 *   You may not reuse this code without permission of the authors unless superseded
 *   by previous licensing. Isn't this stuff complicated? We should hire a code lawyer.
 *
 * Created: Feb 28, 2013
 * Edited:	May 01, 2013
 * Authors:	Ryan E. Guerra (me@ryaneguerra.com)
 * 			Naren Anand (nanand@rice.edu)
 */

#include "driverlib/gpio.h"

#ifndef __WSD_DEFINES_H__
#define __WSD_DEFINES_H__

// ASCII character maps used in terminal processing code
#define ASCII_BS	8
#define ASCII_DEL 	127
#define ASCII_CR	13
#define ASCII_NL 	10
#define ASCII_ESC	27

#define ASCII_0		48
#define ASCII_1		49
#define ASCII_2		50
#define ASCII_3		51
#define ASCII_4		52
#define ASCII_5		53
#define ASCII_6		54
#define ASCII_7		55
#define ASCII_8		56
#define ASCII_9		57


// OpCodes used to define atomic commands that the Stellaris executes.
#define OP_EASTER				97		// a
//#define OP_UNASSIGNED			98		// b
#define OP_SET_RF_CALIB_TABLE	99		// c
#define OP_REGISTER_DUMP		100		// d
#define OP_CLEAR_ERROR			101		// e
#define OP_GETBACKDOORDATA		103 	// g -
#define OP_HELP					104		// h
#define OP_GETCHIPINFO			105		// i
#define OP_GETCHIPINFO_JSON		106		// j
#define OP_TOGGLE_LED			108		// l
#define OP_SET_TX_GAIN			110		// n
#define OP_READ_REG 			114 	// r
#define OP_COMMIT_CALIB_TABLE	115		// s
//#define OP_TOGGLE_PA			116 	// t	REG - disabled, but only commented
#define OP_TOGGLE_CALLOAD		117		// u
#define OP_SILENT_WRITE_REG		118 	// v - fast write
#define OP_RWRITE_REG			119 	// w - robust write
#define OP_TOGGLE_TXRX_CTRL		120		// x
#define OP_SET_RXTXPA_TIMEOUT	121		// y
#define OP_TOGGLE_SW_METHOD		122		// z

#define OP_MACHINE_JSON			7		// BELL

#define OP_PWR_ON_INIT			65 	// A
#define OP_SET_RX_FREQUENCY		66 	// B
#define OP_SET_CHANNEL			67	// C
#define OP_SET_TX_FREQUENCY		68 	// D
#define OP_LOOPBACK_ENABLE		69	// E
#define OP_AUTOCAL_SEQUENCE		70 	// F
#define OP_SET_TX_VGA1			71	// G (0:31)
#define OP_SET_TX_VGA2			72	// H (0:25)
#define OP_SWAP_IQ_PHASE		73  // I
#define OP_SET_RX_VGA1			74	// J (0:127)
#define OP_SET_RX_VGA2			75	// K (0:10)
#define OP_SET_RX_LNAGAIN		76	// L (0:2)
#define OP_SET_RX				77	// M
#define OP_SET_TX				78	// N
#define OP_SET_SERIAL_CODE		79	// O
#define OP_PRINT_LOFT_DCO_VALS	80	// P
//#define OP_START_BB_MODE		81	// Q
#define OP_TOGGLE_SRXEN			82 	// R
#define OP_TOGGLE_STXEN			83 	// S
#define OP_TOGGLE_RXOUTSW		84	// T
#define OP_SET_ADCDAC_MODE		85	// U
#define OP_SET_TX_BW			86 	// V (0:15)
#define OP_SET_RX_BW			87 	// W (0:15)
#define OP_SOFT_RESET			88	// X
#define OP_SELECT_TRX_MODE		90	// Z REG - used for arbitrary test cases

// Some bit defines to make some operations more readable
#define BIT0					0x00000001
#define BIT1					0x00000002
#define BIT2					0x00000004
#define BIT3					0x00000008
#define BIT4					0x00000010
#define BIT5					0x00000020
#define BIT6					0x00000040
#define BIT7					0x00000080

// Some math defines
#define UL_MAX	4294967295 //2^32 - 1
enum {
	TRUE = 1,
	FALSE = 0
};
// Used to signal directions.
typedef enum {
	TX = 1,
	RX = 0
} Xmission_Dir_t;
// Used to signal state
typedef enum {
	ENABLE = 1,
	DISABLE = 0
} En_Dis_t;
// Used to flag the autocalibration sequence what the transmission
// mode currently is.
typedef enum {
	LMS_CAL_NORMAL = 0,
	LMS_CAL_RF_LOOPBACK = 1,
	LMS_CAL_BB_LOOPBACK = 2
} AutoCalMode_t;

// Used to select between Tx/Rx switching modes
typedef enum {
	TxRxSWMode_FDD = 0,
	TxRxSWMode_TDD = 1
} TxRxSWMode_t;

typedef enum {
	EXTERNAL_LNA = 0,
	INTERNAL_LNA = 1
} RFInputSrc_t;

// Used to select where Rx input comes from: either external LNAs or
//

// Non-volatile User Flash Register Values
// NOTE: Once committed, this ARE NOT re-writable without a terrible
//       reset procedure that I will break your arms if you make me
//       implement.
#define STEL_USER_REG0				0x400FE1E0
#define STEL_USER_REG1				0x400FE1E4
#define STEL_USER_REG2				0x400FE1E8
#define STEL_USER_REG3				0x400FE1EC
#define STEL_USER_REG_MASK			0x7FFFFFFF
#define STEL_USER_NOTREAD_MASK		0x80000000

// from hw_flash.h
//#define FLASH_USERREG0          0x400FE1E0  // User Register 0
//#define FLASH_USERREG1          0x400FE1E4  // User Register 1
//#define FLASH_USERREG2          0x400FE1E8  // User Register 2
//#define FLASH_USERREG3          0x400FE1EC  // User Register 3

// Pin/Bank assignments for RF Control lines
// DO NOT EDIT WITHOUT TALKING TO RYAN!!!!
// YOU COULD BRICK YOUR RADIO!!!!
#define CTRL_UHF_PA1_PIN		GPIO_PIN_4
#define CTRL_UHF_PA2_PIN		GPIO_PIN_5
#define CTRL_WIFI_PA_PIN		GPIO_PIN_7
#define CTRL_UHF_ANT_PIN		GPIO_PIN_4
#define CTRL_WIFI_ANT_PIN		GPIO_PIN_6
#define CTRL_UHF_LNA_PIN		GPIO_PIN_5

#define CTRL_UHF_PA1_BASE		GPIO_PORTE_BASE

#define CTRL_UHF_PA2_BASE		GPIO_PORTC_BASE
#define CTRL_WIFI_PA_BASE		GPIO_PORTC_BASE
#define CTRL_ANT_BASE			GPIO_PORTC_BASE

#define CTRL_UHF_LNA_BASE		GPIO_PORTB_BASE

#define CTRL_FPGA_TXEN_PIN		GPIO_PIN_6
#define CTRL_FPGA_NMI_PIN		GPIO_PIN_7

#define CTRL_FPGA_BASE			GPIO_PORTB_BASE
#define CTRL_FPGA_SOFTEN_MASK	0x32	// DSM RST inactive, EN TOP, 4W SPI

// Masks used when formatting commands for the Lime LMS6002D
#define LMS_WRITE_CMD_MASK 		0x00008000
#define LMS_ADDR_MASK 			0x0000007F
#define LMS_DATA_MASK 			0x000000FF

// Important Lime Microsystems LMS6002D register definitions
//TODO - not all register definitions are here
#define LMS_TOP_RCCAL_R			0x01
#define LMS_CHIP_INFO_R 		0x04
#define LMS_TOP_SOFTEN_R		0x05
#define LMS_TOP_LPFCAL_1_R		0x06
#define LMS_TOP_LPFCAL_2_R		0x07
#define LMS_LOOPBACK_CTRL_R		0x08
#define LMS_CLK_CTRL_R			0x09
#define LMS_FDDTDD_CTRL_R		0x0A
#define LMS_TOP_MISC_CONFIG_R	0x0B
#define LMS_DCOCOM_TXLPF_R		0x3F
#define LMS_GAIN_TXVGA1_R		0x41
#define LMS_DCO_TXRF_I_R		0x42
#define LMS_DCO_TXRF_Q_R		0x43
#define LMS_LOOPBACK_PA_R 		0x44
#define LMS_TXVGA2_CTRL_R		0x44
#define LMS_GAIN_TXVGA2_R		0x45
#define LMS_BB_LOOPB_R			0x46
#define LMS_TXRF_TXLOBUF_R		0x47
#define LMS_TXRF_TXMIX_R		0x48
#define LMS_TXRF_TXDRV_R		0x49
#define LMS_SPARE_CONFIG_R_1	0x4F 	// used for connectivity testing
#define LMS_DAC_CTRL_R			0x57
#define LMS_ADC_CTRL_1_R		0x58
#define LMS_ADC_CTRL_2_R		0x59
#define LMS_MISC_CTRL_R			0x5A
#define LMS_DCOCOM_RXLPF_R		0x5F
#define LMS_RXVGA2_CTRL_R		0x64
#define LMS_GAIN_RXVGA2_R		0x65
#define LMS_RXVGA2_PD_R			0x66
#define LMS_DCOCOM_RXVGA2_R 	0x6E
#define LMS_SPARE_CONFIG_R_2  	0x6F 	// used for hard reset detection
#define LMS_RXFE_CTRL_R			0x70
#define LMS_IN1SEL_MIX_R		0x71
#define LMS_DCO_RXFE_I_R		0x71
#define LMS_INLOAD_CTRL_R		0x72
#define LMS_DCO_RXFE_Q_R		0x72
#define LMS_RXFE_IP2TRIM_I_R	0x73
#define LMS_RXFE_IP2TRIM_Q_R	0x74
#define LMS_LNA_CTRL_R			0x75
#define LMS_GAIN_RXVGA1_R		0x76
#define LMS_LNA_RDLINT_R		0x79
#define LMS_MIX_RXFE_R			0x7C	// register for ext LNA input enable
#define LMS_LNA_DIRECT_R		0x7D	// magic register used in LMS's RF LoopB GUI

#define LMS_BASE_MASK			0xF0
#define LMS_TOP_BASE			0x00 //
#define LMS_TXPLL_BASE			0x10
#define LMS_RXPLL_BASE			0x20
#define LMS_TXLPF_BASE			0x30 //
#define LMS_TXRF_BASE			0x40
#define LMS_RXLPF_BASE			0x50 //
#define LMS_DACADC_BASE			0x50
#define LMS_RXVGA2_BASE			0x60 //
#define LMS_RXFE_BASE			0x70

// Lime Microsystems LMS6002D Base Register Offsets
#define LMS_DCO_RESULT_OFFSET	0x00
#define LMS_DCO_STATUS_OFFSET	0x01
#define LMS_DCO_CNTVAL_OFFSET	0x02
#define LMS_DCO_CALIB_OFFSET	0x03
#define LMS_LPF_BWC_OFFSET		0x04
#define LMS_DCO_DACCAL_OFFSET	0x05
#define LMS_RCCAL_OFFSET		0x06
#define LMS_DCOCMP_PD_OFFSET	0x0F

#define LMS_PLL_NINT_OFFSET			0x00	// NINT[8:1]
#define LMS_PLL_NFRAC_A_OFFSET		0x01	// NINT[0], NFRAC[22:16]
#define LMS_PLL_NFRAC_B_OFFSET		0x02	// NFRAC[15:8]
#define LMS_PLL_NFRAC_C_OFFSET		0x03	// NFRAC[7:0]
#define LMS_PLL_CTRL_OFFSET			0x04
#define LMS_PLL_FREQSEL_OFFSET		0x05
#define LMS_PLL_ICHP_OFFSET			0x06
#define LMS_PLL_OFFUP_OFFSET		0x07
#define LMS_PLL_VOVCOREG_OFFSET		0x08
#define LMS_PLL_OFFDOWN_OFFSET		0x08
#define LMS_PLL_VCOCAP_OFFSET		0x09
#define LMS_PLL_VTUNE_OFFSET		0x0A
#define LMS_PLL_VCOCOMP_PD_OFFSET	0x0B

// Masks for the LMS_CHIP_INFO_R register
#define LMS_VERSION_MASK 		0x000000F0
#define LMS_REVISION_MASK		0x0000000F

// Masks for the LMS_TOP_LPFCAL_1_R (0x06) register
#define LMS_LPFCAL_1_CLKSEL_M			BIT3
#define LMS_LPFCAL_1_PD_CLK_M			BIT2
#define LMS_LPFCAL_1_ENF_EN_CAL_M		BIT1
#define LMS_LPFCAL_1_RST_CAL_M			BIT0

// Masks for the LMS_TOP_LPFCAL_2_R (0x07) register
#define LMS_LPFCAL_2_EN_CAL_M			BIT7
#define LMS_LPFCAL_2_FORCE_CODE_CAL_M	(BIT6|BIT5|BIT4)
#define LMS_LPFCAL_2_BWC_M				(BIT3|BIT2|BIT1|BIT0)

// Address of various DC Calibration modules
#define LMS_DC_LPF_TUNE_A		0x00000000 //default
#define LMS_DC_TXRXLPF_I_A		0x00000000 //default, TX/RX are the same
#define LMS_DC_TXRXLPF_Q_A		0x00000001
#define LMS_DC_RXVGA_DC_REF_A 	0x00000000 //default
#define LMS_DC_VGA2A_I_A		0x00000001
#define LMS_DC_VGA2A_Q_A		0x00000002
#define LMS_DC_VGA2B_I_A		0x00000003
#define LMS_DC_VGA2B_Q_A		0x00000004

// Masks for the LMS_CLK_CTRL_R (0x09)
// default 01000000
#define LMS_CLK_EN_RXOUTSW				BIT7
#define LMS_CLK_EN_PLLCLKOUT			BIT6
#define LMS_CLK_EN_LPFCAL				BIT5
#define LMS_CLK_EN_RX_VGA2_DCCAL		BIT4
#define LMS_CLK_EN_RX_LPF_DCCAL			BIT3
#define LMS_CLK_EN_RX_DSM_SPI			BIT2
#define LMS_CLK_EN_TX_LPF_DCCAL			BIT1 //note: changed name from RX LPF SPI DCCAL
#define LMS_CLK_EN_TX_DSM_SPI			BIT0

// Masks for the LMS_BB_LOOPB_R (0x46) Register
#define LMS_PKDBW_M					(BIT7|BIT6|BIT5|BIT4)	// default: 0000 max BW
#define LMS_LOOPBBEN_M				(BIT3|BIT2)				// default: 00; Switch closed -> 11
#define LMS_FST_PKDET_M				BIT1					// default: 0
#define LMS_FST_TXHFBIAS_M			BIT0					// default: 0

// Masks for the LMS_RCCAL_OFFSET (0x56, 0x36) registers
#define LMS_RCCAL_DACBUF_PD_M		BIT7
#define LMS_RCCAL_LPF_M				(BIT6|BIT5|BIT4)
//#define LMS_RCCAL_DCOCMP_PD_M		BIT3	// ERRATA: this no longer exists
#define LMS_RCCAL_DCODAC_LPF_PD_M	BIT2
#define LMS_RCCAL_DCOREF_LPF_PD_M	BIT1
#define LMS_FIL_LPF_PD_M			BIT0

#define LMS_VGA2A_PD_DCO_COMP_M		BIT3
#define LMS_VGA2B_PD_DCO_COMP_M		BIT1

// Masks/consts for the LMS_RXFE_CTRL_R register
#define LMS_RXFE_DECODE_DISABLE_M	BIT1
#define LMS_RXFE_ENABLE_M			BIT0

#define LMS_RXFE_TEST_MODE_ENABLE	(BIT1|BIT0)
#define LMS_RXFE_TEST_MODE_DISABLE	BIT0

// Masks for the LMS_DC_RESULT_OFFSET register
#define LMS_DC_RESULT_REGVAL_M		(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)

// Masks for the LMS_DCO_DACCAL_OFFSET register
// default 00001100
#define LMS_BYP_EN_LPF_M			BIT6
#define LMS_DCO_DACCAL_M			(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)

// Masks & consts for LMS_DCO_RESULT_OFFSET (+0x0) register
#define LMS_DCO_RESULT_M			(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)

#define LMS_DCO_RESULT_INVALID_HI	0x1F	// 31 decimal, see 023a_FAQ_v1.0r8.pdf #4.7
#define LMS_DCO_RESULT_INVALID_LO	0x00	// ^ second time starts at 0
#define LMS_DCO_CNTVAL_M			(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)

// Masks for LMS_TOP_RCCAL_R (0x01) register
#define LMS_RCCAL_LPFCAL_M			(BIT7|BIT6|BIT5)
#define LMS_DC_STATUS_LOCK_M		(BIT4|BIT3|BIT2)
#define	LMS_DC_STATUS_CLBR_BUSY_M	BIT1
#define LMS_DC_STATUS_UD_M			BIT0

// Field masks for LMS_DCO_CALIB_OFFSET register (+0x03)
#define LMS_DC_CALIB_START_CLBR		BIT5
#define LMS_DC_CALIB_LOAD			BIT4
#define LMS_DC_CALIB_SRESET			BIT3
#define LMS_DC_CALIB_ADDR			(BIT2|BIT1|BIT0)

#define LMS_DC_START_CLBR_M			BIT5
#define LMS_DC_LOAD_M				BIT4
#define LMS_DC_SRESET_M				BIT3
#define LMS_DC_ADDR_M				(BIT2|BIT1|BIT0)

// Field masks for LMS_DCO_RXFE_X_R registers X=I/Q
// LMS_DCO_RXFE_Q_R, LMS_DCO_RXFE_I_R
// Also known as LMS_IN1SEL_MIX_R
#define LMS_IN1SEL_MIX_RXFE_M		BIT7
#define LMS_INLOAD_LNA_RXFE_M		BIT7
#define LMS_DCO_RXFE_M				(BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_DCO_RXFE_SIGN_M			BIT6
#define LMS_DCO_RXFE_MAG_M			(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_DCO_TXRF_SIGN_M			BIT7
#define LMS_DCO_TXRF_MAG_M			(BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
typedef enum {
	EXTINPUT_2_LNABUFF = 0,
	LNA_2_LNABUFF = 1
} IN1SELState_t;

// Field masks for LMS_MISC_CTRL_R (0x5A)
#define LMS_MISC_RX_FSYNC_POL_M			BIT7
#define LMS_MISC_RX_INTERLEAVE_M		BIT6
#define LMS_MISC_DAC_CLK_EDGE_POL_M		BIT5
#define LMS_MISC_TX_FSYNC_POL_M			BIT4
#define LMS_MISC_TX_INTERLEAVE_M		BIT3
#define LMS_MISC_ADC_SAMPLE_PHASE_SEL_M	BIT2
#define LMS_MISC_CLK_NON_OVERLAP_ADJ_M	(BIT1|BIT0)

// Masks and consts for LMS_LNA_DIRECT_R register
#define LMS_LNA_DIRECT_PD_TIA_RXFE_M	BIT3
#define LMS_LNA_DIRECT_PD_MXLOB_RXFE_M	BIT2
#define LMS_LNA_DIRECT_PD_MIX_RXFE_M	BIT1
#define LMS_LNA_DIRECT_PD_M				BIT0

#define LMS_LNA_ENABLE					0x00	// See FAQ 5.9
#define LMS_LNA_DISABLE					BIT0

// Field masks for LMS_LOOPBACK_CTRL
#define LMS_LOOPBACK_LPFIN_M		BIT6	// do not use individually
#define LMS_LOOPBACK_VGA2IN_M		BIT5	// do not use individually
#define LMS_LOOPBACK_OPIN_M			BIT4	// do not use individually
#define LMS_LOOPBACK_LBBBEN_M		(BIT6|BIT5|BIT4)
#define LMS_LOOPBACK_LBRFEN_M		(BIT3|BIT2|BIT1|BIT0)

// Mask definitions for reg LMS_TXVGA2_CTRL_R
#define LMS_LOOPBACK_AUX_PA_GAIN_M	(BIT6|BIT5|BIT4|BIT3)
#define LMS_TXVGA2_CTRL_M			(BIT4|BIT3)
#define LMS_TXVGA2_USR_CTRL_M		(BIT4|BIT3)		 //the subset of CTRL that isn't reserved

// Cal Guide 1.1r2 was updated in 1.1r4 to shift
// this assignment from BIT1 to BIT2. We've verified
// that the new bit definition IS correct.
#define LMS_LOOPBACK_PD_DRVAUX_M	BIT2
//#define LMS_LOOPBACK_PD_DRVAUX_M	BIT1
//#define LMS_LOOPBACK_PD_PKDET		BIT0

#define LMS_RF_LOOPB_DISABLED		0
#define LMS_RF_LOOPB_LNA_1			1	// UHF
#define LMS_RF_LOOPB_LNA_2			2	// 2.4 GHz WiFi
#define LMS_RF_LOOPB_LNA_3			3	// Broadband Input-Only

#define LMS_BB_LOOPB_DISABLED		0
#define LMS_BB_LOOPB_LPFIN			BIT6
#define LMS_BB_LOOPB_VGA2IN			BIT5
#define LMS_BB_LOOPB_OPIN			BIT4

// Field definitions for reg LMS_TXVGA2_CTRL_R
#define LMS_TXVGA2_PA1OFF_PA2OFF	0
#define LMS_TXVGA2_PA1ON_PA2OFF		BIT3	// UHF
#define LMS_TXVGA2_PA1OFF_PA2ON		BIT4	// WiFi
typedef enum {
	AUXPA_ON = 0,
	AUXPA_OFF = 1
} AUXPAState_t;
#define	LMS_LOOPB_AUXPA_PWRDOWN		BIT2			//default
#define LMS_LOOPB_AUXPA_PWRUP		0x00

// LMS6002D Gain fields
#define LMS_GAIN_TXVGA1_M		(BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_GAIN_TXVGA2_M		(BIT7|BIT6|BIT5|BIT4|BIT3)
#define LMS_GAIN_RXVGA1_M		(BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_GAIN_RXVGA2_M		(BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_GAIN_LNA_M			(BIT7|BIT6)


// Masks for PLL settings fields
#define LMS_PLL_VOVCOREG_LSB_M	BIT7
#define LMS_PLL_VCOCAP_M		(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_PLL_VTUNE_H_M		BIT7
#define LMS_PLL_VTUNE_L_M		BIT6
#define LMS_PLL_FREQSEL_M		(BIT7|BIT6|BIT5|BIT4|BIT3|BIT2)
#define LMS_PLL_FREQSEL_NFRAC_M	(BIT4|BIT3|BIT2)
#define LMS_PLL_SELOUT_M		(BIT1|BIT0)
#define LMS_PLL_PD_VCOCOMP_SX_M	BIT3
#define LMS_PLL_ICHP_M			(BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_PLL_OFFUP_M			(BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_PLL_OFFDOWN_M		(BIT4|BIT3|BIT2|BIT1|BIT0)

#define LMS_PLL_ICHP_1200_uA	0x0C
#define LMS_PLL_OFFUP_30_uA		0x03
#define LMS_PLL_OFFDOWN_0_uA	0x00

// Masks for LMS_MIX_RXFE_R register 0x7C
#define LMS_G_FINE_MIX_RXFE_M	(BIT1|BIT0)
#define LMS_RINEN_MIX_RXFE_M	(BIT2)					// 1 = active
#define LMS_LOBN_MIX_RXFE_M		(BIT6|BIT5|BIT4|BIT3)
typedef enum {
	LOAD_ENABLED = 1,
	LOAD_DISABLED = 0
} RINEN_MIX_RXFE_t;

// Masks for LMS_TOP_MISC_CONFIG_R register 0x0B
#define LMS_TOP_XCO_PD_M		BIT4
#define LMS_TOP_XCO_SLFBIAS_M	BIT3
#define LMS_TOP_XCO_BUFF_BYP_M	BIT2
#define LMS_TOP_DCOREF_LPFCAL_M	BIT1
#define LMS_TOP_RFLOOB_SW_ON_M	BIT0

// LMS6002 LPF Bandwidth settings for LMS_LPFCAL_2_BWC_M
#define LMS_LPF_BW_14		0x00
#define LMS_LPF_BW_10		0x01
#define LMS_LPF_BW_7		0x02
#define LMS_LPF_BW_6		0x03
#define LMS_LPF_BW_5		0x04
#define LMS_LPF_BW_4_375	0x05
#define LMS_LPF_BW_3_5		0x06
#define LMS_LPF_BW_3		0x07
#define LMS_LPF_BW_2_75		0x08
#define LMS_LPF_BW_2_5		0x09
#define LMS_LPF_BW_1_92		0x0A
#define LMS_LPF_BW_1_5		0x0B
#define LMS_LPF_BW_1_375	0x0C
#define LMS_LPF_BW_1_25		0x0D
#define LMS_LPF_BW_0_875	0x0E
#define LMS_LPF_BW_0_75		0x0F

// LMS6002 LMS_DAC_CTRL_R (0x57) field masks
#define LMS_DACADC_EN_M				BIT7
#define LMS_DAC_DECODE_M			BIT6
#define LMS_TX_CTRL_DAC_LOAD_RES_M	(BIT5|BIT4|BIT3)
#define LMS_TX_CTRL_REF_RES_M		BIT2
#define LMS_TX_CTRL_DAC_CURR_M		(BIT1|BIT0)

#define LMS_DAC_TX_CTRL_M	(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)

// LMS6002D LMS_ADC_CTRL_2_R (0x59) field masks
#define LMS_ADC_REF_BIAS_RES_ADJ_M	(BIT7|BIT6)
#define LMS_ADC_REF_BIAS_UP_M		(BIT5|BIT4)
#define LMS_ADC_REF_BIAS_DOWN_M		(BIT3|BIT2|BIT1|BIT0)
#define LMS_ADC_REF_GAIN_ADJ_M		(BIT6|BIT5)
#define LMS_ADC_COMM_MODE_ADJ_M		(BIT4|BIT3)
#define LMS_ADC_REF_BUFF_BOOST_M	(BIT2|BIT1)
#define LMS_ADC_IN_BUFF_DISABLE_M	BIT0

#define LMS_ADC_REFBIAS_10uA		BIT7
#define LMS_ADC_REFGAIN_1_75V		BIT5
#define LMS_ADC_COMMODE_960mV		BIT3

// LMS6002D TXRF field masks
#define LMS_TXRF_ICT_TXLOBUF_M		(BIT7|BIT6|BIT5|BIT4)
#define LMS_TXRF_VBCAS_TXDRV_M		(BIT3|BIT2|BIT1|BIT0)
#define LMS_TXRF_ICT_TXMIX_M		(BIT4|BIT3|BIT2|BIT1|BIT0)
#define LMS_TXRF_ICT_TXDRV_M		(BIT4|BIT3|BIT2|BIT1|BIT0)

#define LMS_TXRF_LOBUF_MAXCURRENT	0xF0
#define LMS_TXRF_LOBUF_GOODCURRENT	0x90	// best in testing
#define LMS_TXRF_ICT_TXMIX_31MA		0x1F
#define LMS_TXRF_ICT_TXMIX_21mA		0x15	// better in testing
#define LMS_TXRF_ICT_TXDRV_31MA		0x1F
#define LMS_TXRF_ICT_TXDRV_22mA		0x16	// better in testing

// LMS6002 LMS_LNA_CTRL_R (0x75) field masks
#define LMS_LNA_CTRL_RXFE_GAIN_M	(BIT7|BIT6)
#define LMS_LNA_CTRL_LNASEL_M		(BIT5|BIT4)
#define LMS_LNA_CTRL_RXFE_CBE_M		(BIT3|BIT2|BIT1|BIT0)
#define LMS_LNA_RDLINT_M			(BIT5|BIT4|BIT3|BIT2|BIT1|BIT0)

#define LMS_LNA_DISABLED			0x00
#define LMS_LNA_1					BIT4
#define LMS_LNA_2					BIT5
#define LMS_LNA_3					(BIT4|BIT5)

#define LMS_LNA_OPTIMAL_BIASRES		0x37	// Lime FAQ 5.27

// Masks for the LMS_LPF_BWC_OFFSET register fields
#define LMS_LPF_BWC_M		(BIT5|BIT4|BIT3|BIT2)	// default 0000 (14)
#define LMS_LPF_LPFEN_M		BIT1					// default: 1
#define LMS_LPF_DECODE		BIT0					// default: 0

// Bitmasks for the LMS_TOP_SOFTEN_R (0x05) Top Level Register
#define LMS_TOP_CONTROL_M		BIT7
#define LMS_TOP_SRESET_PD		BIT5
#define LMS_TOP_EN				BIT4
#define LMS_TOP_STXEN			BIT3
#define LMS_TOP_SRXEN			BIT2
#define LMS_TOP_TFWMODE_M		BIT1

// Bitmasks for the LMS_RXVGA2_CTRL_R register
#define LMS_RXVGA2_CTRL_CMVC_M		(BIT5|BIT4|BIT3|BIT2)
#define LMS_RXVGA2_CTRL_EN_M		BIT1
#define LMS_RXVGA2_CTRL_DECODE_M	BIT0

// Default common-mode voltage setting provided by Lime
// in FAQ_r11, #5.27
#define RXVGA2_CTRL_CMV_DEFAULT		(BIT5|BIT4|BIT2)

// DCO Comparator bitmasks
// Bitmasks for LMS_DCOCOM_TXLPF_R (0x3F)
//				LMS_DCOCOM_RXLPF_R (0x5F)
#define LMS_DCOCOM_TXLPF_EN_M		BIT7
#define LMS_DCOCOM_RXLPF_EN_M		BIT7
#define LMS_DCOCOM_LPF_PD_M			BIT7

// Bitmasks for register LMS_DCOCOM_RXVGA2_R (0x6E)
#define LMS_DCOCOM_RXVGA2B_EN_M		BIT7
#define LMS_DCOCOM_RXVGA2A_EN_M		BIT6

// Flags for setting or clearing bits
#define LMS_SET		1
#define LMS_CLEAR	0

// Flags for Lime_SetChannel() function
// 	Channel Type (US/UK UHF or WIFI)
#define LMS_US_UHF				2	// WARPLab type 1 is 5 GHz Wifi
#define LMS_EU_UHF				3	//
#define LMS_WIFI				0	// WARPLab type 0 is 2.4 GHz WiFi
// 	Transmission Direction
#define LMS_DIRECTION_RX		0
#define LMS_DIRECTION_TX		1
// 	Channel BW - FIXME: should extend to other chan BWs when available.
#define LMS_CHAN_5MHz			5
#define LMS_CHAN_10MHz			10
#define LMS_CHAN_20MHz			20

// Macros for register/field writes - in-line replacement required to make it
// execute quickly and avoid another function call
#define __setRegField(ulReg, ulFieldMask, ulVal, ulBuff) \
		do { \
			Lime_Read(ulReg, &ulBuff); \
			ulBuff &= ~ulFieldMask; \
			ulBuff |= (ulVal & ulFieldMask); \
			Lime_FastWrite(ulReg, ulBuff); \
		} while (0)

#endif //__WSD_DEFINES_H__

// This is down here because we ALWAYS want to define the __ENABLE_USB__ symbol
// contained in this header file under any and all circumstances.
#include "wsd_settings.h"

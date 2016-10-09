/*
 * txrx_switching_macros.h
 *
 *  Created on: Mar 12, 2014
 *      Author: rng
 */

#ifndef TXRX_SWITCHING_MACROS_H_
#define TXRX_SWITCHING_MACROS_H_

// =============================================================================
// Tx/Rx Macros ================================================================
// =============================================================================
// Macros for setup and quickly switching Tx/Rx directions.
// These calls are as flat as possible for speed, and rely on a number
// of globals to avoid Read/Write/Readback operations.

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ENABLE FDD MODE
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Disables TDD mode, and makes sure that all Tx/Rx pathways are active in software
// Older revisions don't have external LNA
#define __LMS_PRIME_TRX_FDD \
	do { \
		Lime_SetSoftEnable(LMS_TOP_SRESET_PD | LMS_TOP_EN | LMS_TOP_STXEN | LMS_TOP_SRXEN); \
		Lime_FastWrite(LMS_FDDTDD_CTRL_R, 0x0); \
		WSD_TurnOffUHFPAs(); \
		WSD_TurnOffWiFiPAs(); \
		if (trx.TRXState == RX_NORMAL) { \
			Lime_SetRxInputPath(INTERNAL_LNA); \
		} else { \
			Lime_SetRxInputPath(EXTERNAL_LNA); \
		} \
	} while (0)

// ===================================================================================
// Generic Macros for Readability, these are flattened to speed switching speed
// ===================================================================================
//#define __SPI_rx_pll_to_iemix1_pad 		SSIDataPutNonBlocking(SSI0_BASE, (LMS_IN1SEL_MIX_R << 8) | LMS_WRITE_CMD_MASK | trx.ulCurrentRx_I_DCO);
// The LMS_IN1SEL_MIX_R register's top bit controls the input to Rx Mixer Buff, while the
// lower 7 bits control Rx I DCO setting. The write mask sets first bit high.
#define __SPI_rx_mixer_to_extlna_pad	SSIDataPutNonBlocking(SSI0_BASE, (LMS_IN1SEL_MIX_R << 8) | LMS_WRITE_CMD_MASK | 0x00                  | trx.ulCurrentRx_I_DCO); \
										trx.RxInputMode = EXTERNAL_LNA;
#define __SPI_rx_mixer_to_intlna_out	SSIDataPutNonBlocking(SSI0_BASE, (LMS_IN1SEL_MIX_R << 8) | LMS_WRITE_CMD_MASK | LMS_IN1SEL_MIX_RXFE_M | trx.ulCurrentRx_I_DCO); \
										trx.RxInputMode = INTERNAL_LNA;

#define __SPI_disable_rx_lnas		SSIDataPutNonBlocking(SSI0_BASE, (LMS_LNA_CTRL_R << 8) | LMS_WRITE_CMD_MASK | (0x40 & (LMS_LNA_CTRL_RXFE_GAIN_M | LMS_LNA_CTRL_LNASEL_M)) );
#define __SPI_disable_rx_lna_1		SSIDataPutNonBlocking(SSI0_BASE, (LMS_LNA_CTRL_R << 8) | LMS_WRITE_CMD_MASK | (0x50 & (LMS_LNA_CTRL_RXFE_GAIN_M | LMS_LNA_CTRL_LNASEL_M)) );
#define __SPI_enable_rx_lna_1		SSIDataPutNonBlocking(SSI0_BASE, (LMS_LNA_CTRL_R << 8) | LMS_WRITE_CMD_MASK | (0x50 & (LMS_LNA_CTRL_RXFE_GAIN_M | LMS_LNA_CTRL_LNASEL_M)) );
#define __SPI_enable_rx_lna_2		SSIDataPutNonBlocking(SSI0_BASE, (LMS_LNA_CTRL_R << 8) | LMS_WRITE_CMD_MASK | (0xE0 & (LMS_LNA_CTRL_RXFE_GAIN_M | LMS_LNA_CTRL_LNASEL_M)) );


#define __wifi_pa_on 				GPIOPinWrite(CTRL_WIFI_PA_BASE, CTRL_WIFI_PA_PIN, 0xFF);
#define __wifi_pa_off 				GPIOPinWrite(CTRL_WIFI_PA_BASE, CTRL_WIFI_PA_PIN, 0x00);

#define __wifi_tx_to_ant_1			GPIOPinWrite(CTRL_ANT_BASE, CTRL_WIFI_ANT_PIN , 0x00);
#define __wifi_tx_to_ant_2			GPIOPinWrite(CTRL_ANT_BASE, CTRL_WIFI_ANT_PIN , 0xFF);

#define __uhf_pa_2_on				GPIOPinWrite(CTRL_UHF_PA2_BASE, CTRL_UHF_PA2_PIN, 0xFF);
#define __uhf_pa_2_off				GPIOPinWrite(CTRL_UHF_PA2_BASE, CTRL_UHF_PA2_PIN, 0x00);

#define __uhf_pa_1_on				GPIOPinWrite(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN, 0xFF);
#define __uhf_pa_1_off				GPIOPinWrite(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN, 0x00);

#define __uhf_ext_lna_on			GPIOPinWrite(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN, 0xFF);
#define __uhf_ext_lna_off			GPIOPinWrite(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN, 0x00);

#define __uhf_tx_to_ant_1			GPIOPinWrite(CTRL_ANT_BASE, CTRL_UHF_ANT_PIN, 0x00);
#define __uhf_tx_to_ant_2			GPIOPinWrite(CTRL_ANT_BASE, CTRL_UHF_ANT_PIN, 0xFF);

#define __block_for_spi_writes		while (SSIBusy(SSI0_BASE)==true) {/*block*/}
#define __SPI_flush_one_spi_rx			SSIDataGetNonBlocking(SSI0_BASE, &g_ulDevNull);

// ===================================================================================
// SET TX FDD
// ===================================================================================
// The WiFi version disables UHF PAs, switches WiFi antenna, and enables WiFi PA.
#define __LMS_SET_TX_FDD \
	do { \
		if (trx.isTransmittingInUHFBand) { \
			__uhf_ext_lna_off \
			if (trx.TRXState == RX_NORMAL) { \
				__SPI_rx_mixer_to_extlna_pad \
			} else { \
				__SPI_rx_mixer_to_intlna_out \
			} \
			__SPI_disable_rx_lnas \
			__wifi_pa_off \
			__wifi_tx_to_ant_2 \
			__uhf_tx_to_ant_1 \
			__uhf_pa_2_on \
			__uhf_pa_1_on \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
			__SPI_flush_one_spi_rx \
		}  else { \
			__SPI_rx_mixer_to_intlna_out \
			__SPI_disable_rx_lnas \
			__uhf_pa_2_off \
			__uhf_pa_1_off \
			__uhf_ext_lna_off \
			__uhf_tx_to_ant_2 \
			__wifi_tx_to_ant_1 \
			__wifi_pa_on \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
			__SPI_flush_one_spi_rx \
		} \
	} while (0)

// ===================================================================================
// SET RX FDD
// ===================================================================================
// FDD mode Rx enable

// The WiFi version disables WiFi PA, re-enables PA2, and switches WiFi switch
#define __LMS_SET_RX_FDD \
	do { \
		if (trx.isTransmittingInUHFBand) { \
			__uhf_pa_2_off \
			__uhf_pa_1_off \
			__wifi_pa_off \
			__uhf_ext_lna_on \
			if (trx.TRXState == RX_NORMAL) { \
				__SPI_enable_rx_lna_1 \
				__NOP_DELAY_LOOP(trx.ulTxRxPAOffTimeout); \
				__SPI_rx_mixer_to_intlna_out \
			} else { \
				__SPI_disable_rx_lna_1 \
				__NOP_DELAY_LOOP(trx.ulTxRxPAOffTimeout); \
				__SPI_rx_mixer_to_extlna_pad \
			} \
			__uhf_tx_to_ant_2 \
			__wifi_tx_to_ant_2 \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
			__SPI_flush_one_spi_rx \
		} else { \
			__wifi_pa_off \
			__uhf_ext_lna_off \
			__uhf_pa_2_off \
			__uhf_pa_1_off \
			__SPI_enable_rx_lna_2 \
			__NOP_DELAY_LOOP(trx.ulTxRxPAOffTimeout); \
			__wifi_tx_to_ant_2 \
			__uhf_tx_to_ant_2 \
			__SPI_rx_mixer_to_intlna_out \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
			__SPI_flush_one_spi_rx \
		}; \
	} while (0)


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ENABLE TDD MODE
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enables TDD mode, and sets to Rx mode. Makes sure all RF pathways are
// active in software. Disable all Tx PAs for safety

#define __LMS_PRIME_TRX_TDD \
	do { \
		Lime_SetSoftEnable(LMS_TOP_SRESET_PD | LMS_TOP_EN | LMS_TOP_STXEN | LMS_TOP_SRXEN); \
		Lime_FastWrite(LMS_FDDTDD_CTRL_R, 0x3); \
		WSD_TurnOffUHFPAs(); \
		WSD_TurnOffWiFiPAs(); \
		Lime_SetRxInputPath(INTERNAL_LNA); \
	} while (0)

// ===================================================================================
// SET TX TDD
// ===================================================================================
// TDD mode Tx enable
// 1. Set TDD mode to transmit: 0x0A = b00000010
// 2. Turn off WiFi PA (just to be sure)
// 3. Delay to let Tx PLL settle before turning on Tx PAs.
// 4. Switch Tx => ANT1
// 5. Turn on UHF PA2;
// 6. Turn on UHF PA1;
// 7. Wait for SPI Writes to finish, just in case
// 8. Clear SPI RX buffers
//
// The WiFi version disables UHF PAs, switches WiFi antenna, enables WiFi PA.
#define __LMS_SET_TX_TDD \
	do { \
		if (trx.isTransmittingInUHFBand) { \
			SSIDataPutNonBlocking(SSI0_BASE, (LMS_FDDTDD_CTRL_R << 8) | LMS_WRITE_CMD_MASK | 0x2); \
			__wifi_pa_off \
			__uhf_ext_lna_off \
			__NOP_DELAY_LOOP(trx.ulTxRxPAOffTimeout); \
			__uhf_tx_to_ant_1 \
			__uhf_pa_2_on \
			__uhf_pa_1_on \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
		} else { \
			SSIDataPutNonBlocking(SSI0_BASE, (LMS_FDDTDD_CTRL_R << 8) | LMS_WRITE_CMD_MASK | 0x2); \
			__uhf_pa_2_off \
			__uhf_pa_1_off \
			__uhf_ext_lna_off \
			__NOP_DELAY_LOOP(trx.ulTxRxPAOffTimeout); \
			__uhf_tx_to_ant_2 \
			__wifi_tx_to_ant_1 \
			__wifi_pa_on \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
		}; \
	} while (0)

// ===================================================================================
// SET RX TDD
// ===================================================================================
// TDD Mode Rx enable
// 1. Set TDD mode to transmit: 0x0A = b00000011
// 2. Disable Tx PA2
// 3. Disable Tx PA1
// 4. Delay to let Tx PA2 to transition to off state, and Rx PLL to
//    settle, before enabling the Rx pathway
// 5. Switch Rx => ANT1;
// 6. Wait for SPI Writes to finish, just in case
// 7. Clear SPI RX buffers
//
// The WiFi version disables WiFi PA, and switches WiFi antenna
#define __LMS_SET_RX_TDD \
	do { \
		if (trx.isTransmittingInUHFBand) { \
			SSIDataPutNonBlocking(SSI0_BASE, (LMS_FDDTDD_CTRL_R << 8) | LMS_WRITE_CMD_MASK | 0x3); \
			__wifi_pa_off \
			__uhf_pa_2_off \
			__uhf_pa_1_off \
			__NOP_DELAY_LOOP(trx.ulRxTxPLLSettlingTimeout); \
			__wifi_tx_to_ant_2 \
			__uhf_tx_to_ant_2 \
			__uhf_ext_lna_on \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
		} else { \
			SSIDataPutNonBlocking(SSI0_BASE, (LMS_FDDTDD_CTRL_R << 8) | LMS_WRITE_CMD_MASK | 0x3); \
			__wifi_pa_off \
			__uhf_pa_2_off \
			__uhf_pa_1_off \
			__uhf_ext_lna_off \
			__NOP_DELAY_LOOP(trx.ulRxTxPLLSettlingTimeout); \
			__wifi_tx_to_ant_2 \
			__uhf_tx_to_ant_2 \
			__block_for_spi_writes \
			__SPI_flush_one_spi_rx \
		}; \
	} while (0)

#endif /* TXRX_SWITCHING_MACROS_H_ */

/*
 * volo_gpio.c
 *
 * Contains all helper functions for functional GPIO pins on the Stellaris
 * microcontroller. Primarily RF Control lines and LEDs.
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
 *  Created on: March 14, 2013
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 *
 */

#include "include/wsd_gpio.h"


//*****************************************************************************
//
// Turn on the given PA. NOT SAFE. DO NOT USE.
//
// \param ulPABase The GPIO base for the pin controlling the target PA.
// \param ulCtrlPin The bit-packed representation of the pin controlling the
//                  target PA.
//
//*****************************************************************************
void
WSD_TurnOnPA(unsigned long ulPABase, unsigned long ulCtrlPin)
{
	GPIOPinWrite(ulPABase, ulCtrlPin, 0xFF);
}

//*****************************************************************************
//
// Turn off the given PA. NOT SAFE. DO NOT USE.
//
// \param ulPABase The GPIO base for the pin controlling the target PA.
// \param ulCtrlPin The bit-packed representation of the pin controlling the
//                  target PA.
//
//*****************************************************************************
void
WSD_TurnOffPA(unsigned long ulPABase, unsigned long ulCtrlPin)
{
	GPIOPinWrite(ulPABase, ulCtrlPin, 0x00);
}

//*****************************************************************************
//
// Simple call to turn ON the WiFi PA safely. The WiFi PA's activation is
// mutually exclusive to the UHF PA's operation due to the power budget of the
// board. The second-stage UHF PA and the WiFi PA cannot be active
// simultaneously.
//
//*****************************************************************************
void
WSD_TurnOnWiFiPAs()
{
	WSD_TurnOffUHFPAs();
	WSD_TurnOnPA(CTRL_WIFI_PA_BASE, CTRL_WIFI_PA_PIN);
}

//*****************************************************************************
//
// Simple call to turn ON the UHF PAs safely. The UHF PAs' activation is
// mutually exclusive to the WiFi PA's operation due to the power budget of the
// board. The second-stage UHF PA and the WiFi PA cannot be active
// simultaneously.
//
// Also, since we don't want to drive an inactive PA with a strong signal, we
// turn the UHF PAs on outside to inside.
//
//*****************************************************************************
void
WSD_TurnOnUHFPAs()
{
	WSD_TurnOffWiFiPAs();
	WSD_TurnOnPA(CTRL_UHF_PA2_BASE, CTRL_UHF_PA2_PIN);
	WSD_TurnOnPA(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN);
}

//*****************************************************************************
//*****************************************************************************
void
WSD_TurnOnUHFLNA()
{
	WSD_TurnOnPA(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN);
}

//*****************************************************************************
//*****************************************************************************
void
WSD_TurnOffUHFLNA()
{
	WSD_TurnOffPA(CTRL_UHF_LNA_BASE, CTRL_UHF_LNA_PIN);
}

//*****************************************************************************
//
// Simple call to turn ON the UHF PA1 safely.
//
//*****************************************************************************
void
WSD_TurnOnUHF_PA_1()
{
	WSD_TurnOffWiFiPAs();
	WSD_TurnOnPA(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN);
}

//*****************************************************************************
//
// Simple call to turn OFF the UHF PAs safely. Since we don't want to drive an
// inactive PA with a strong signal, we turn them off inside to outside.
//
//*****************************************************************************
void
WSD_TurnOffUHFPAs()
{
	WSD_TurnOffPA(CTRL_UHF_PA1_BASE, CTRL_UHF_PA1_PIN);
	WSD_TurnOffPA(CTRL_UHF_PA2_BASE, CTRL_UHF_PA2_PIN);
}

//*****************************************************************************
//
// Simple call to turn OFF the WiFi PA safely. Nothing special is required.
//
//*****************************************************************************
void
WSD_TurnOffWiFiPAs()
{
	WSD_TurnOffPA(CTRL_WIFI_PA_BASE, CTRL_WIFI_PA_PIN);
}

//*****************************************************************************
//
// Set the target switch so that TX => ANT1 and RX <= ANT2.
//
// \param ulSWBase The GPIO base for the pin controlling the target switch.
// \param ulCtrlPin The bit-packed representation of the pin controlling the
//                  target switch.
//
//*****************************************************************************
void
WSD_RFSwitch_TX_ANT1(unsigned long ulSWBase, unsigned long ulCtrlPin)
{
	GPIOPinWrite(ulSWBase, ulCtrlPin, 0x00);
}

//*****************************************************************************
//
// Set the target switch so that TX => ANT2 and RX <= ANT1.
//
// \param ulSWBase The GPIO base for the pin controlling the target switch.
// \param ulCtrlPin The bit-packed representation of the pin controlling the
//                  target switch.
//
//*****************************************************************************
void
WSD_RFSwitch_RX_ANT1(unsigned long ulSWBase, unsigned long ulCtrlPin)
{
	GPIOPinWrite(ulSWBase, ulCtrlPin, 0xFF);
}

//*****************************************************************************
//
// Turn on Green LED indicator on PA7
//
//*****************************************************************************
void
Stel_TurnOffHeartbeatLED(void)
{
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x80);
}

//*****************************************************************************
//
// Turn off Green LED indicator on PA7
//
//*****************************************************************************
void
Stel_TurnOnHeartbeatLED(void)
{
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00);
}

//*****************************************************************************
//
// Toggle LED indicator on PA7. The LED is active-low.
//
//*****************************************************************************
void
Stel_ToggleHeartBeatLED(void)
{
	unsigned long ulVal;

	// Read which value it currently is
	ulVal = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
	if (ulVal)
	{
		// the LED is active-low
		Stel_TurnOnHeartbeatLED();
	}
	else
	{
		Stel_TurnOffHeartbeatLED();
	}
}

//*****************************************************************************
//
// Turn on Red LED indicator on PA7. The LED is active-low.
//
//*****************************************************************************
void
Stel_TurnOnErrorLED(void)
{
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0x00);
}

//*****************************************************************************
//
// Turn off Red LED indicator on PA7. The LED is active-low.
//
//*****************************************************************************
void
Stel_TurnOffErrorLED(void)
{
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0xFF);
}



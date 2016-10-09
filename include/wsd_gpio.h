/*
 * wsd_gpio.h
 *
 * Contains all helper prototypes for functional GPIO pins on the Stellaris
 * microcontroller. Primarily RF Control lines and LEDs.
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

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "wsd_defines.h"
#include "driverlib/gpio.h"

#ifndef __WSD_GPIO_H__
#define __WSD_GPIO_H__

void WSD_TurnOnPA(unsigned long ulPABase, unsigned long ulCtrlPin);
void WSD_TurnOffPA(unsigned long ulPABase, unsigned long ulCtrlPin);
void WSD_TurnOnWiFiPAs(void);
void WSD_TurnOnUHFPAs(void);
void WSD_TurnOffUHFPAs(void);
void WSD_TurnOnUHF_PA_1(void);
void WSD_TurnOffWiFiPAs(void);
void WSD_TurnOnUHFLNA(void);
void WSD_TurnOffUHFLNA(void);
void WSD_RFSwitch_TX_ANT1(unsigned long ulSWBase, unsigned long ulCtrlPin);
void WSD_RFSwitch_RX_ANT1(unsigned long ulSWBase, unsigned long ulCtrlPin);
void Stel_TurnOffHeartbeatLED(void);
void Stel_TurnOnHeartbeatLED(void);
void Stel_ToggleHeartBeatLED(void);

void Stel_TurnOnErrorLED(void);
void Stel_TurnOffErrorLED(void);

#endif //__WSD_GPIO_H__

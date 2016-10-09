/*
 * usb_helpers.h
 *
 *
 * Contains all helper functions for USB driver
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 *   CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  Created on: Jul 17, 2012
 *      Author: Narendra Anand (nanand@rice.edu)
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 */

#include "stdlib.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "include/wsd_defines.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/ustdlib.h"

#include "string.h"
#include "utils/flash_pb.h"
//#include "wsd_main.h"

#ifdef __ENABLE_UART__
	#include "driverlib/uart.h"
	#include "utils/uartstdio.h"
#endif //__ENABLE_UART__

#ifdef __ENABLE_USB__
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "wsd_usb_serial_structs.h"

#ifndef __VOLO_USB_H__
#define __VOLO_USB_H__

static char usb_pkt_sent;

tBoolean VoloSetLineCoding(tLineCoding *psLineCoding);

void GetLineCoding(tLineCoding *psLineCoding);
unsigned long
Stel_USBControlHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData);
unsigned long
Stel_USBTxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData);
unsigned long
Stel_USBRxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData);

#endif //__ENABLE_USB__
#endif //__VOLO_USB_H__

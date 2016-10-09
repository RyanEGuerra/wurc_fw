//*****************************************************************************
//
// usb_serial_structs.h - Data structures defining this USB CDC device.
//
// Copyright (c) 2009-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 8555 of the EK-LM3S9B92 Firmware Package.
//
//*****************************************************************************


#include "inc/hw_types.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "include/wsd_defines.h"

#ifndef __USB_SERIAL_STRUCTS_H__
#define __USB_SERIAL_STRUCTS_H__

// The size of the transmit and receive buffers used for the redirected UART.
// This number should be a power of 2 for best performance.  256 is chosen
// pretty much at random though the buffer should be at least twice the size of
// a maxmum-sized USB packet.
#define UART_BUFFER_SIZE 256

#ifdef __ENABLE_USB__

// Driver buffers for the USB UART peripheral
extern const tUSBBuffer g_sTxBuffer;
extern const tUSBBuffer g_sRxBuffer;
extern const tUSBDCDCDevice g_sCDCDevice;
extern unsigned char g_pucUSBTxBuffer[];
extern unsigned char g_pucUSBRxBuffer[];

// USB UART callback handlers
extern unsigned long Stel_USBRxHandler(void *pvCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);
extern unsigned long Stel_USBTxHandler(void *pvlCBData, unsigned long ulEvent,
                               unsigned long ulMsgValue, void *pvMsgData);

#endif // __ENABLED_USB__
#endif // __USB_SERIAL_STRUCTS_H__

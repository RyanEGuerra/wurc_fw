//*****************************************************************************
//
// usb_serial_structs.c - Data structures defining this CDC USB device.
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

#include "wsd_usb_serial_structs.h"


#ifdef __ENABLE_USB__

//*****************************************************************************
//
// The languages supported by this device.
//
//*****************************************************************************
const unsigned char g_pLangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

//*****************************************************************************
//
// The manufacturer string.
//
//*****************************************************************************
const unsigned char g_pManufacturerString[] =
{
    2 + (18 * 2),
    USB_DTYPE_STRING,
    'V', 0, 'o', 0, 'l', 0, 'o', 0, ' ', 0, 'W', 0, 'i', 0, 'r', 0, 'e', 0,
    'l', 0, 'e', 0, 's', 0, 's', 0, ' ', 0, 'L', 0, 'L', 0, 'C', 0, '.', 0,
};

//*****************************************************************************
//
// The product string.
//
//*****************************************************************************
const unsigned char g_pProductString[] =
{
    2 + (34 * 2),
    USB_DTYPE_STRING,
    'V', 0,
    'o', 0,
    'l', 0,
    'o', 0,
    ' ', 0,
    'W', 0,
    'i', 0,
    'r', 0,
    'e', 0,
    'l', 0,
    'e', 0,
    's', 0,
    's', 0,
    ' ', 0,
    'W', 0,
    'S', 0,
    'D', 0,
    ' ', 0,
    'S', 0,
    'e', 0,
    'r', 0,
    'i', 0,
    'a', 0,
    'l', 0,
    ' ', 0,
    'I', 0,
    'n', 0,
    't', 0,
    'e', 0,
    'r', 0,
    'f', 0,
    'a', 0,
    'c', 0,
    'e', 0,
};

//*****************************************************************************
//
// The serial number string.
//
//*****************************************************************************
const unsigned char g_pSerialNumberString[] =
{
    2 + (10 * 2),
    USB_DTYPE_STRING,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '9', 0,
    '0', 0,
    '0', 0,
    '1', 0
};

//*****************************************************************************
//
// The control interface description string.
//
//*****************************************************************************
const unsigned char g_pControlInterfaceString[] =
{
    2 + (21 * 2),
    USB_DTYPE_STRING,
    'A', 0, 'C', 0, 'M', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0,
    'r', 0, 'o', 0, 'l', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

//*****************************************************************************
//
// The configuration description string.
//
//*****************************************************************************
const unsigned char g_pConfigString[] =
{
    2 + (26 * 2),
    USB_DTYPE_STRING,
    'S', 0, 'e', 0, 'l', 0, 'f', 0, ' ', 0, 'P', 0, 'o', 0, 'w', 0,
    'e', 0, 'r', 0, 'e', 0, 'd', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0,
    'f', 0, 'i', 0, 'g', 0, 'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0,
    'o', 0, 'n', 0
};

//*****************************************************************************
//
// The descriptor string table.
//
//*****************************************************************************
const unsigned char * const g_pStringDescriptors[] =
{
    g_pLangDescriptor,
    g_pManufacturerString,
    g_pProductString,
    g_pSerialNumberString,
    g_pControlInterfaceString,
    g_pConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_pStringDescriptors) /                \
                                sizeof(unsigned char *))

//*****************************************************************************
//
// CDC device callback function prototypes.
//
//*****************************************************************************
unsigned long Stel_USBRxHandler(void *pvCBData, unsigned long ulEvent,
                        unsigned long ulMsgValue, void *pvMsgData);
unsigned long Stel_USBTxHandler(void *pvCBData, unsigned long ulEvent,
                        unsigned long ulMsgValue, void *pvMsgData);
unsigned long Stel_USBControlHandler(void *pvCBData, unsigned long ulEvent,
                             unsigned long ulMsgValue, void *pvMsgData);

//*****************************************************************************
//
// The CDC device initialization and customization structures. In this case,
// we are using USBBuffers between the CDC device class driver and the
// application code. The function pointers and callback data values are set
// to insert a buffer in each of the data channels, transmit and receive.
//
// With the buffer in place, the CDC channel callback is set to the relevant
// channel function and the callback data is set to point to the channel
// instance data. The buffer, in turn, has its callback set to the application
// function and the callback data set to our CDC instance structure.
//
//*****************************************************************************
tCDCSerInstance g_sCDCInstance;			// USB driver workspace (stack memory allocated
										// to this driver instance)

extern const tUSBBuffer g_sTxBuffer;	// USB buffer descriptors
extern const tUSBBuffer g_sRxBuffer;	// USB buffer descriptors

const tUSBDCDCDevice g_sCDCDevice =
{										//RYAN - pg. 83 of SW-USBL-UG-5228 describes these
    USB_VID_STELLARIS,					// vendor id
    USB_PID_SERIAL,						// product id
    0,									// power consumption in mA
    USB_CONF_ATTR_SELF_PWR,				// power attribute (pwr_m, self, bs, rwake)
    Stel_USBControlHandler,				// Control Event Handler Callback
    (void *)&g_sCDCDevice,				// value passed to Control handler w/ each call
    USBBufferEventCallback,				// USB Receive Event Handler Callback
    (void *)&g_sRxBuffer,				// value passed to Receive handler w/ each call
    USBBufferEventCallback,				// USB Transmit Event Handler Callback
    (void *)&g_sTxBuffer,				// value passed to Transmit handler w/ each call
    g_pStringDescriptors,				// pointer to string table (manufacture, dev id, serial, etc...)
    NUM_STRING_DESCRIPTORS,				// number of entries in previous table
    &g_sCDCInstance						// pointer to the workspace for this driver instance
};

//*****************************************************************************
//
// Receive buffer (from the USB perspective).
//
//*****************************************************************************
unsigned char g_pcUSBRxBuffer[UART_BUFFER_SIZE];
unsigned char g_pucRxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sRxBuffer =
{
    false,                          // This is a receive buffer.
    Stel_USBRxHandler,               // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketRead,              // pfnTransfer
    USBDCDCRxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pcUSBRxBuffer,                // pcBuffer
    UART_BUFFER_SIZE,               // ulBufferSize
    g_pucRxBufferWorkspace          // pvWorkspace
};

//*****************************************************************************
//
// Transmit buffer (from the USB perspective).
//
//*****************************************************************************
unsigned char g_pcUSBTxBuffer[UART_BUFFER_SIZE];
unsigned char g_pucTxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sTxBuffer =
{
    true,                           // This is a transmit buffer.
    Stel_USBTxHandler,              // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketWrite,             // pfnTransfer
    USBDCDCTxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pcUSBTxBuffer,                // pcBuffer
    UART_BUFFER_SIZE,               // ulBufferSize
    g_pucTxBufferWorkspace          // pvWorkspace
};
#endif //__ENABLE_USB__

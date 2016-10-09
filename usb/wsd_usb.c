/*
 * wsd_usb.c
 *
 * Contains all helper functions for USB driver, both reference and custom.
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

#include "wsd_usb.h"
#include "include/wsd_main.h"

#ifndef __ENABLE_USB__
#warning INFO - USB0 is NOT enabled
#endif // __ENABLE_USB__
#ifdef __ENABLE_USB__

// Count the number of times that "Set Line Coding" has been called. We found
// that when using Mac OS X Screen utility to connect to the device, this was
// always called twice upon a connect. Thus, this is a passive, hacky way to
// detect USB connections without monitoring VBUS.
char cUSB_SetLineCount_EventCounter = 0;

//*****************************************************************************
//
// Set the communication parameters to use on the UART.
// This function call is a remnant from the reference code that handled the USB
// interrupt for setting the serial line coding. Handling that interrupt is
// required by the CDC Driver API, but we don't actually have to do anything
// RYAN - this should probably be removed or cut out of the codebase.
//
//*****************************************************************************
tBoolean
WSDSetLineCoding(tLineCoding *psLineCoding)
{
#ifdef __DEBUG__
    unsigned long ulConfig;
#endif //__DEBUG__
    tBoolean bRetcode;

    //
    // Assume everything is OK until a problem is detected.
    //
    bRetcode = true;

#ifdef __DEBUG__
    //
    // Word length.  For invalid values, the default is to set 8 bits per
    // character and return an error.
    //
    switch(psLineCoding->ucDatabits)
    {
        case 5:
        {
            ulConfig = UART_CONFIG_WLEN_5;
            break;
        }

        case 6:
        {
            ulConfig = UART_CONFIG_WLEN_6;
            break;
        }

        case 7:
        {
            ulConfig = UART_CONFIG_WLEN_7;
            break;
        }

        case 8:
        {
            ulConfig = UART_CONFIG_WLEN_8;
            break;
        }

        default:
        {
            ulConfig = UART_CONFIG_WLEN_8;
            bRetcode = false;
            break;
        }
    }

    //
    // Parity.  For any invalid values, set no parity and return an error.
    //
    switch(psLineCoding->ucParity)
    {
        case USB_CDC_PARITY_NONE:
        {
            ulConfig |= UART_CONFIG_PAR_NONE;
            break;
        }

        case USB_CDC_PARITY_ODD:
        {
            ulConfig |= UART_CONFIG_PAR_ODD;
            break;
        }

        case USB_CDC_PARITY_EVEN:
        {
            ulConfig |= UART_CONFIG_PAR_EVEN;
            break;
        }

        case USB_CDC_PARITY_MARK:
        {
            ulConfig |= UART_CONFIG_PAR_ONE;
            break;
        }

        case USB_CDC_PARITY_SPACE:
        {
            ulConfig |= UART_CONFIG_PAR_ZERO;
            break;
        }

        default:
        {
            ulConfig |= UART_CONFIG_PAR_NONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Stop bits.  The hardware only supports 1 or 2 stop bits whereas CDC
    // allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
    // invalid or unsupported value of ucStop, set up for 1 stop bit but return
    // an error in case the caller needs to Stall or otherwise report this back
    // to the host.
    //
    switch(psLineCoding->ucStop)
    {
        //
        // One stop bit requested.
        //
        case USB_CDC_STOP_BITS_1:
        {
            ulConfig |= UART_CONFIG_STOP_ONE;
            break;
        }

        //
        // Two stop bits requested.
        //
        case USB_CDC_STOP_BITS_2:
        {
            ulConfig |= UART_CONFIG_STOP_TWO;
            break;
        }

        //
        // Other cases are either invalid values of ucStop or values that are
        // not supported, so set 1 stop bit but return an error.
        //
        default:
        {
            ulConfig = UART_CONFIG_STOP_ONE;
            bRetcode = false;
            break;
        }
    }
#endif //__DEBUG__

    //
    // Set the UART mode appropriately.
    //
//RYAN - this should just not do anything since the UART0 will not be operational
//       in the final firmware release.
//    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(),
//                            psLineCoding->ulRate, ulConfig);

    //
    // Let the caller know if a problem was encountered.
    //
    return(bRetcode);
}

//*****************************************************************************
//
// Get the communication parameters in use on the UART.
// This is required by the CDC Driver API (or rather, the interrupt must be handled),
// but I never saw it get called during a normal USB attach/detach sequence. In
// any case, this function returns static values that just so happen to be our
// preferred operating values.
// RYAN - this function should probably be removed from the codebase.
//
//*****************************************************************************
void
GetLineCoding(tLineCoding *psLineCoding)
{
    //unsigned long ulConfig;
    //unsigned long ulRate;

    //RYAN - All we're required to do is to return a valid line coding when asked.
    psLineCoding->ulRate = 115200;
    psLineCoding->ucDatabits = 8;
    psLineCoding->ucParity = USB_CDC_PARITY_NONE;
    psLineCoding->ucStop = USB_CDC_STOP_BITS_1;
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the USB device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the notification event.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
Stel_USBControlHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
               void *pvMsgData)
{
    // Which event was sent?
    switch(ulEvent)
    {
    // The host has connected. We flush the USB buffers in this case to remove any
    // gobbledegook.
        case USB_EVENT_CONNECTED:
        {
#ifdef __DEBUG_UART__
        	PrintStrUARTDebug("> USB Connected\r\n");
#endif //__DEBUG_UART__

        	// Flush the USB buffers.
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            break;
        }
    // The host has disconnected. Since the USB VBUS is not connected and the WSD
    // is a self-powered device, this will actually never get called, lolol...
    // To "fix" that, we'd have to connect a GPIO to the VBUS and monitor that pin.
        case USB_EVENT_DISCONNECTED:
        {
#ifdef __DEBUG_UART__
        	PrintStrUARTDebug("> USB DIS-Connected\r\n");
#endif //__DEBUG_UART__
            break;
        }
    // Return the current serial communication parameters.
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
        	// modifies the passed struct to set the required line coding params
        	// current just sets 115200 8-N-1, which should be default
            GetLineCoding(pvMsgData);
            break;
        }
    // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_LINE_CODING:
        {
#ifdef __DEBUG_UART__
        	PrintStrUARTDebug("> SLC Event\r\n");
#endif //__DEBUG_UART__
        	cUSB_SetLineCount_EventCounter++;
            //WSDSetLineCoding(pvMsgData);
        	if (cUSB_SetLineCount_EventCounter == 2)
			{
        		// reset the slc_counter so that the computer can dis-connect and
        		// then reconnect the terminal and we'll re-post the splash
        		//TODO - this section assumes that the SetLineCoding command is always sent twice.
        		//       while that is true for Mac OS X Screen utility, it may not be true for other terminal
        		//       emulators. This needs to be checked.
        		cUSB_SetLineCount_EventCounter = 0;

        		// signal to the main loop that the splash page should be printed
        		// to the USB terminal.
        		Stel_SetResetTerminalPendingFlag();
#ifdef __DEBUG_UART__
        		PrintStrUARTDebug("> Resetting USB...\r\n");
#endif //__DEBUG_UART__
			}
            break;
        }
    // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        {
            break;
        }
    // Send a break condition on the serial line.
        case USBD_CDC_EVENT_SEND_BREAK:
        {
            //SendBreak(true);
            break;
        }
    // Clear the break condition on the serial line.
        case USBD_CDC_EVENT_CLEAR_BREAK:
        {
            //SendBreak(false);
            break;
        }
    // Ignore SUSPEND and RESUME for now.
        case USB_EVENT_SUSPEND:
        {
#ifdef __DEBUG_UART__
        	PrintStrUARTDebug("\r\n> USB Suspended\r\n");
#endif //__DEBUG_UART__
        	break;
        }
        case USB_EVENT_RESUME:
        {
#ifdef __DEBUG_UART__
        	PrintStr("\r\n> USB Resumed\r\n");
#endif //__DEBUG_UART__
            break;
        }
    // Other events can be safely ignored.
        default:
        {
            break;
        }
    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the USB transmit channel
// (data to the USB host).
//
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the notification event.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
Stel_USBTxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    // Which event was sent?
    switch(ulEvent)
    {
    // Transmit was complete
        case USB_EVENT_TX_COMPLETE:
        {
#ifdef __DEBUG__
        	//UARTprintf(" TX\n");
#endif //__DEBUG__
            // There is nothing to do here since it is handled by the
            // USBBuffer.
            break;
        }
    // Other events can be safely ignored.
        default:
        {
            break;
        }
    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the USB receive channel
// (data from the USB host).
//
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the notification event.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
Stel_USBRxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    unsigned long ulCount;
    // Which event was sent?
    switch(ulEvent)
    {
    // A new packet has been received.
        case USB_EVENT_RX_AVAILABLE:
        {
        	// Signal to the main loop that a character is waiting in the USB
        	// Rx buffer.
        	Stel_SetUSB0BufferHasCharsFlag();
            break;
        }
	// This is a request for how much unprocessed data is still waiting to
	// be processed. Used for flow control. So we return the size of the current
    // USB Rx Queue.
        case USB_EVENT_DATA_REMAINING:
        {
        	// How much data is available in the buffer?
        	ulCount = USBBufferDataAvailable(&g_sRxBuffer);
            return(ulCount);
        }
	// This is a request for a buffer into which the next packet can be
	// read.  This mode of receiving data is not supported so let the
	// driver know by returning 0.  The CDC driver should not be sending
	// this message but this is included just for illustration and
	// completeness.
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

    // Other events can be safely ignored.
        default:
        {
            break;
        }
    }

    return(0);
}

#endif //__ENABLE_USB__

/*
 * wsd_startup_ccs.c
 *
 *  Modified startup_ccs.c specifically for the AGC Calib board and WSD for both
 *  lm3s5y36 and lm3s5r36 versions.
 *  Assumes that the hardware platform is the lm3s5X36 with associated peripherals.
 *
 *  A note on the USB device interrupt vector: the USB0DeviceIntHandler function is
 *  defined in the low-level USB driver library linked to this project. Our API to
 *  USB device is a CDC Device Class Driver API that makes RxHandler, TxHandler, and
 *  ControlHandler our callback functions to handle device interrupts on the three USB
 *  channels instantiated by the CDC Class Driver: Rx, Tx, and Control. See pg. 80 of
 *  SW-USBL-UG-5228 for more details about this implementation.
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
 *   This file is adapted from templates provided by TI's StellarisWare package
 *   available here: http://www.ti.com/tool/sw-lm3s
 *
 *  Created on: Feb 26, 2013
 *  	Author: Texas Instruments (license follows)
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 *
 * From TI's code, we copy the header and license agreement; our code meets these
 * terms because we only target Stellaris hardware and all other non-TI code has
 * been written by either Ryan or Naren from scratch.
 *
 * / *****************************************************************************
 * /
 * / startup_ccs.c - Startup code for use with TI's Code Composer Studio.
 * /
 * / Copyright (c) 2009-2012 Texas Instruments Incorporated.  All rights reserved.
 * / Software License Agreement
 * /
 * / Texas Instruments (TI) is supplying this software for use solely and
 * / exclusively on TI's microcontroller products. The software is owned by
 * / TI and/or its suppliers, and is protected under applicable copyright
 * / laws. You may not combine this software with "viral" open-source
 * / software in order to form a larger program.
 * /
 * / THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * / NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * / NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * / A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * / CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * / DAMAGES, FOR ANY REASON WHATSOEVER.
 * /
 * / This is part of revision 8555 of the EK-LM3S9B92 Firmware Package.
 * /
 * / *****************************************************************************
 *
 */

#include "inc/hw_types.h"
#include "utils/uartstdio.h"

#include "include/wsd_defines.h"
#include "include/wsd_main.h"

#ifndef __ENABLE_USB__
#warning INFO - USB0 NOT enabled.
#endif

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern void _c_int00(void);

//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern unsigned long __STACK_TOP;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
extern void USB0DeviceIntHandler(void);
extern void UART0IntHandler(void);
extern void GPIO_PortB_IntHandler(void);
//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
// RYAN - check page 94 of lm3s5y36 data sheet for a list of these
//
//*****************************************************************************
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((unsigned long)&__STACK_TOP),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    IntDefaultHandler,                      // The MPU fault handler
    IntDefaultHandler,                      // The bus fault handler
    IntDefaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    IntDefaultHandler,                      // The PendSV handler
    IntDefaultHandler,                      // The SysTick handler
    IntDefaultHandler,                      // GPIO Port A
    GPIO_PortB_IntHandler,                  // GPIO Port B
    IntDefaultHandler,                      // GPIO Port C
    IntDefaultHandler,                      // GPIO Port D
    IntDefaultHandler,                      // GPIO Port E
    UART0IntHandler,						// UART0 Rx and Tx Enabled
    IntDefaultHandler,                      // UART1 Rx and Tx
    IntDefaultHandler,                      // SSI0 Rx and Tx
    IntDefaultHandler,                      // I2C0 Master and Slave
    IntDefaultHandler,                      // PWM Fault
    IntDefaultHandler,                      // PWM Generator 0
    IntDefaultHandler,                      // PWM Generator 1
    IntDefaultHandler,                      // PWM Generator 2
    IntDefaultHandler,                      // Quadrature Encoder 0
    IntDefaultHandler,                      // ADC Sequence 0
    IntDefaultHandler,                      // ADC Sequence 1
    IntDefaultHandler,                      // ADC Sequence 2
    IntDefaultHandler,                      // ADC Sequence 3
    IntDefaultHandler,                      // Watchdog timer
    IntDefaultHandler,                      // Timer 0 subtimer A
    IntDefaultHandler,                      // Timer 0 subtimer B
    IntDefaultHandler,                      // Timer 1 subtimer A
    IntDefaultHandler,                      // Timer 1 subtimer B
    IntDefaultHandler,                      // Timer 2 subtimer A
    IntDefaultHandler,                      // Timer 2 subtimer B
    IntDefaultHandler,                      // Analog Comparator 0
    IntDefaultHandler,                      // Analog Comparator 1
    0,                      				// (RESERVED) Analog Comparator 2
    IntDefaultHandler,                      // System Control (PLL, OSC, BO)
    IntDefaultHandler,                      // FLASH Control
    0,                      				// (RESERVED) GPIO Port F
    0,                      				// (RESERVED) GPIO Port G
    0,                      				// (RESERVED) GPIO Port H
    IntDefaultHandler,                      // UART2 Rx and Tx
    IntDefaultHandler,                      // SSI1 Rx and Tx
    0,                      				// (RESERVED) Timer 3 subtimer A
    0,                      				// (RESERVED) Timer 3 subtimer B
    IntDefaultHandler,                      // I2C1 Master and Slave
    0,                      				// (RESERVED) Quadrature Encoder 1
    IntDefaultHandler,                      // CAN0
    0,                     					// (RESERVED) CAN1
    0,                      				// (RESERVED) CAN2
    0,                      				// (RESERVED) Ethernet
    IntDefaultHandler,                      // Hibernate
#ifdef __ENABLE_USB__
    USB0DeviceIntHandler,                   // USB0 - NOTE that this callback is defined in the low-level USB library
#else
    IntDefaultHandler,                   // USB0 - disabled
#endif //__ENABLE_USB__
    0,                      				// (RESERVED) PWM Generator 3
    IntDefaultHandler,                      // uDMA Software Transfer
    IntDefaultHandler,                      // uDMA Error
    IntDefaultHandler,                      // ADC1 Sequence 0
    IntDefaultHandler,                      // ADC1 Sequence 1
    IntDefaultHandler,                      // ADC1 Sequence 2
    IntDefaultHandler,                      // ADC1 Sequence 3
    0,                      				// (RESERVED) I2S0
    0,                      				// (RESERVED) External Bus Interface 0
    0,                      				// (RESERVED) GPIO Port J
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    // Jump to the CCS C initialization routine.
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    // Enter an infinite loop.
    while(1)
    {

    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
//	__newline
//	PrintStr("!FAULT!");
    // Enter an infinite loop.
    while(1)
    {

    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    // Go into an infinite loop.
    while(1)
    {
    }
}

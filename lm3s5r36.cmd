/******************************************************************************
 *
 * Default Linker Command file for the Texas Instruments LM3S5R36
 *
 * This is part of revision 9385 of the Stellaris Peripheral Driver Library.
 *
 *****************************************************************************/

--retain=g_pfnVectors

MEMORY
{
	// This edited linker command reserves the last flash page for persistent
	// parameter memory. Pages are 1024 = 0x400; 0x40000 - 0xC00 = 0x0003F400
	// Synchronize with: FLASH_PB_START and FLASH_PB_END in wsd_settings.h
    // FLASH (RX) : origin = 0x00000000, length = 0x00040000
    FLASH (RX) : origin = 0x00000000, length = 0x0003E000
    SRAM (RWX) : origin = 0x20000000, length = 0x0000c000
}

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=8191                                                          */
/* --library=rtsv7M3_T_le_eabi.lib                                           */

/* Section allocation in memory */

SECTIONS
{
    .intvecs:   > 0x00000000
    .text   :   > FLASH
    .const  :   > FLASH
    .cinit  :   > FLASH
    .pinit  :   > FLASH

    .vtable :   > 0x20000000
    .data   :   > SRAM
    .bss    :   > SRAM
    .sysmem :   > SRAM
    .stack  :   > SRAM
}

//RYAN - Stellaris LM3S5R36 has SRAM from 0x2000.0000 to 0x2000.BFFF
//       BFFF = 49151 in decimal, thus we have 49151 theoretical stack
//       size limit. We limit this in order to try to catch memory overflow
//		 early in testing, rather than later. But it DOES have to hold our
//		 calibration table structure,
// RYAN: 45908 works okay. 20000 ok
__STACK_TOP = __stack + 30000;

/*
 * volo_term.h
 *
 *	To use the Volo terminal in your code, you need to import
 *	this header file, define a term_buff_t struct somewhere, and then
 *	initialize that terminal struct with volo_initTermBuff(..). The
 *  function point argument is what the terminal will use to print
 *  output to the terminal screen.
 *
 *  Finally, one must define and fill out a volo_executeCommand(..)
 *  function to handle complete commands accepted by the terminal.
 *  Opcode checking is up to you: it's a lot easier if the OPSIZE is
 *  1, but you can have arbitrary numbers based on the #defines below.
 *
 *  A couple things: you may want to change the mblaze_nt_types.h include
 *  to some other file for your architecture that defines the uint32_t type.
 *
 *   THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 *   NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 *   NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VOLO WIRELESS, LLC SHALL NOT,
 *   UNDER ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *   DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  Created on: August 16, 2013
 *      Author: Ryan E. Guerra (me@ryaneguerra.com)
 */



#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "inc/hw_types.h"
#include "wsd_defines.h"

#ifndef __VOLO_TERM_H__
#define __VOLO_TERM_H__

/* ========================== Constants ==========================*/
#define VOLO_TERM_OPSIZE		1
#define VOLO_TERM_BUFFERSIZE	30

#define ASCII_BS	8
#define ASCII_DEL 	127
#define ASCII_CR	13
#define ASCII_NL 	10
#define ASCII_ESC	27

/* ====================== Struct Definitions ======================*/
// Terminal Buffer Struct - stores characters and callbacks specific
// to the
typedef struct term_buff_t TerminalBuff_t;
struct term_buff_t {
	// Obvious.
	int numCharsInBuff;
	// Storage for the OpCode characters
	// The extra index is to allow for trailing null terminators
	char cOpCodeBuff[VOLO_TERM_OPSIZE + 1];
	// Storage for the command word characters
	char cCmdBuff[VOLO_TERM_BUFFERSIZE + 1];
	// This callback function should take a char* argument and return a
	// 0 upon success, or 1 upon failure.
	// Usage: retVal = (*vTermPrintCharCallback)(charArg);
	void (*vTermPrintCharCallback)(char *);
	// This pointer to another buffer allows the user to implement a
	// terminal history of linked buffers, if they would like.
	// I'm not going to bother with this at this time. REG.
	TerminalBuff_t *lastBuffer;
};

/* =========================== Macros ============================= */
//Because otherwise this statement just looks awful everywhere.
#define __terminalPrint(B, Str) \
	(*B->vTermPrintCharCallback)(Str)

/* ================= External Function Prototypes ================= */
extern void volo_executeCommand(TerminalBuff_t* pBuff);

/* ===================== Function Prototypes ====================== */
void volo_initTermBuff(TerminalBuff_t* pBuff, void (*printChar)(char *));
void volo_resetCommandBuffer(TerminalBuff_t* pBuff);
void volo_processChar(TerminalBuff_t* pBuff, char newChar);
void volo_printPrompt(TerminalBuff_t* pBuff);

int util_test_cmd_len(TerminalBuff_t *pBuff, int numExpected);
void util_fix9_0_to_str(uint32_t val, char *pcBuff);
void util_fix10_0_to_str(uint32_t val, char *pcBuff);
uint32_t util_str_to_fix9_0(char *pcBuff);
uint32_t util_str_to_fix10_0(char *pcBuff);
uint32_t util_hexStr_to_int(char *pcBuff, size_t numChars);

extern void Stel_PrintStatusJSON(TerminalBuff_t* pBuff);

#endif //__VOLO_TERM_H__

/**
* Copyright (c) Mipot S.p.A. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file
* @date
* @version
*
*/

#ifndef COMM_COMM_H_
#define COMM_COMM_H_

/*******************************************************************************
 * Included files
 ******************************************************************************/
#include "stdint.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define IFC_UART					0

/*!
 * Package application data buffer size
 */
#define COMM_BUF_MAX_LEN			260
#define CMD_HEADER_BYTE				0xAA

#define CMD_SET_LPM					0x70

#define U8_UART_BAUDRATE_9600_VALUE							(uint8_t)(0x00)
#define U8_UART_BAUDRATE_19200_VALUE						(uint8_t)(0x01)
#define U8_UART_BAUDRATE_38400_VALUE						(uint8_t)(0x02)
#define U8_UART_BAUDRATE_57600_VALUE						(uint8_t)(0x03)
#define U8_UART_BAUDRATE_115200_VALUE						(uint8_t)(0x04)
#define U8_UART_BAUDRATE_MIN_VALUE							U8_UART_BAUDRATE_9600_VALUE
#define U8_UART_BAUDRATE_MAX_VALUE							U8_UART_BAUDRATE_115200_VALUE
#define U8_UART_BAUDRATE_DEF_VALUE							U8_UART_BAUDRATE_115200_VALUE

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern uint8_t CommInterface;
extern uint8_t *CommTxBuf;
extern uint8_t *CommRxBuf;
extern uint32_t *CommRxStringLen;
extern uint8_t *CommRxBusy;
extern uint8_t *CommTxBusy;
extern uint8_t *CommTxAsyncBuf;

extern uint8_t uartBaudRateSel;
extern uint32_t shouldUpdateUartSpeed;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void commSetup (void);
void commManager (void);
void commTimer (void);

int isCommIdle (void);

#endif /* COMM_COMM_H_ */

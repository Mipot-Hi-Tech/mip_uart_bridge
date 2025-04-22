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

#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_

/*******************************************************************************
 * Included files
 ******************************************************************************/
#include "usart.h"
#include "comm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SERIAL_BUF_MAX_LEN	260

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern volatile uint8_t U1RxBuf[SERIAL_BUF_MAX_LEN];
extern volatile uint8_t U1RxBusy;
extern volatile uint8_t LPURxBuf[SERIAL_BUF_MAX_LEN];
extern volatile uint8_t LPURxBusy;
extern volatile uint32_t UartRxStringLen;

extern volatile uint8_t UartTxBuf[SERIAL_BUF_MAX_LEN];
extern volatile uint8_t UartTxBusy;

extern volatile uint8_t UartTxAsyncBuf[];

extern USART_TypeDef *usedUart;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void initUarts(void);
void updateUartSpeed(void);
void U1RxCallback(void);
void LPURxCallback(void);
void serialTxCallback(USART_TypeDef *USARTx);
void serialTxCompleteCallback(USART_TypeDef *USARTx);
void serialTimer(void);
void startTxUart(uint32_t len);
void wakeUpUart (void);
void sleepUart (void);

#endif /* SERIAL_SERIAL_H_ */

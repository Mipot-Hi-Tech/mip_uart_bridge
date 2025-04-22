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

/*******************************************************************************
 * Included files
 ******************************************************************************/
#include "usart.h"
#include "serial.h"

#include "comm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART_TIMEOUT_MS		50

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint8_t U1RxBuf[SERIAL_BUF_MAX_LEN];
volatile uint8_t U1RxBusy = 0;
volatile uint8_t LPURxBuf[SERIAL_BUF_MAX_LEN];
volatile uint8_t LPURxBusy = 0;

volatile uint32_t UartRxStringLen = 0;

volatile uint8_t UartTxBuf[SERIAL_BUF_MAX_LEN];
volatile uint8_t UartTxBusy = 0;

volatile uint8_t UartTxAsyncBuf[SERIAL_BUF_MAX_LEN];

static volatile uint32_t U1RxByteCounter = 0;
static volatile uint32_t LPURxByteCounter = 0;
static uint32_t UartTxStringLen = 0;
static uint32_t UartTxByteCounter = 0;
static uint32_t UartRxTimeoutTimer = 0;

USART_TypeDef *usedUart;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*-----------------------------------------------------------------------------
Name      :  initUarts
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void initUarts(void)
{
	uartBaudRateSel = U8_UART_BAUDRATE_DEF_VALUE;

	updateUartSpeed();
	usedUart = NULL;
}

/*-----------------------------------------------------------------------------
Name      :  updateUartSpeed
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void updateUartSpeed(void)
{
	LL_USART_InitTypeDef USART_InitStruct = {0};
	LL_LPUART_InitTypeDef LPUART_InitStruct = {0};
	uint32_t baud;

	if (uartBaudRateSel == U8_UART_BAUDRATE_9600_VALUE)
	{
		baud = 9600;
	}
	else if (uartBaudRateSel == U8_UART_BAUDRATE_19200_VALUE)
	{
		baud = 19200;
	}
	else if (uartBaudRateSel == U8_UART_BAUDRATE_38400_VALUE)
	{
		baud = 38400;
	}
	else if (uartBaudRateSel == U8_UART_BAUDRATE_57600_VALUE)
	{
		baud = 57600;
	}
	else if (uartBaudRateSel == U8_UART_BAUDRATE_115200_VALUE)
	{
		baud = 115200;
	}
	if (LL_USART_IsEnabled(USART1) == 1)
	{
		NVIC_DisableIRQ(USART1_IRQn);
		LL_USART_Disable(USART1);
		LL_USART_DeInit(USART1);

		USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
		USART_InitStruct.BaudRate = baud;
		USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
		USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
		USART_InitStruct.Parity = LL_USART_PARITY_NONE;
		USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
		USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
		USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
		LL_USART_Init(USART1, &USART_InitStruct);
		LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
		LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
		LL_USART_DisableFIFO(USART1);
		LL_USART_ConfigAsyncMode(USART1);

		LL_USART_Enable(USART1);
		NVIC_EnableIRQ(USART1_IRQn);
		/* Polling USART1 initialisation */
		while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
		{
		}
		LL_USART_RequestRxDataFlush(USART1);
		LL_USART_ClearFlag_TXFE(USART1);
		LL_USART_ClearFlag_TC(USART1);
		LL_USART_EnableIT_RXNE_RXFNE(USART1);
	}

	if(LL_LPUART_IsEnabled(LPUART1) == 1)
	{
		NVIC_DisableIRQ(LPUART1_IRQn);
		LL_LPUART_Disable(LPUART1);
		LL_LPUART_DeInit(LPUART1);

		LPUART_InitStruct.PrescalerValue = LL_LPUART_PRESCALER_DIV2;
		LPUART_InitStruct.BaudRate = baud;
		LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
		LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
		LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
		LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
		LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
		LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
		LL_LPUART_SetTXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
		LL_LPUART_SetRXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
		LL_LPUART_SetTXRXSwap(LPUART1, LL_LPUART_TXRX_SWAPPED);

		LL_LPUART_Enable(LPUART1);
		NVIC_EnableIRQ(LPUART1_IRQn);
		/* Polling LPUART1 initialisation */
		while((!(LL_LPUART_IsActiveFlag_TEACK(LPUART1))) || (!(LL_LPUART_IsActiveFlag_REACK(LPUART1))))
		{
		}
		LL_LPUART_RequestRxDataFlush(LPUART1);
		LL_LPUART_ClearFlag_TXFE(LPUART1);
		LL_LPUART_ClearFlag_TC(LPUART1);
		LL_LPUART_EnableIT_RXNE_RXFNE(LPUART1);
	}
}

/*-----------------------------------------------------------------------------
Name      :  U1RxCallback
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void U1RxCallback(void)
{
	static unsigned int rxDataLen = 0;
	uint8_t rxData;

	rxData = LL_USART_ReceiveData8(USART1);
	U1RxBuf[U1RxByteCounter] = rxData;
	if((U1RxByteCounter != 0) || ((rxData == 0xAA) && (U1RxByteCounter == 0)))
	{
		U1RxByteCounter++;
		U1RxBusy = 1;
	}
	if(U1RxByteCounter == 3)
	{
		rxDataLen = rxData;
	}
	if(U1RxByteCounter >= (rxDataLen + 4))
	{
		UartRxStringLen = U1RxByteCounter;
		U1RxByteCounter = 0;
		rxDataLen = 0;
		U1RxBusy = 0;

		if (usedUart != USART1)
		{
			usedUart = USART1;
			CommInterface = IFC_UART;
			CommRxBuf = (uint8_t *) U1RxBuf;
			CommTxBuf = (uint8_t *) UartTxBuf;
			CommRxStringLen = (uint32_t *) &UartRxStringLen;
			CommRxBusy = (uint8_t *) &U1RxBusy;
			CommTxBusy = (uint8_t *) &UartTxBusy;
			CommTxAsyncBuf = (uint8_t *) UartTxAsyncBuf;
		}
	}

	UartRxTimeoutTimer = UART_TIMEOUT_MS;
}

/*-----------------------------------------------------------------------------
Name      :  LPURxCallback
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void LPURxCallback (void)
{
	static unsigned int rxDataLen = 0;
	uint8_t rxData;

	rxData = LL_USART_ReceiveData8(LPUART1);
	LPURxBuf[LPURxByteCounter] = rxData;
	if((LPURxByteCounter != 0) || ((rxData == 0xAA) && (LPURxByteCounter == 0)))
	{
		LPURxByteCounter++;
		LPURxBusy = 1;
	}
	if(LPURxByteCounter == 3)
	{
		rxDataLen = rxData;
	}
	if(LPURxByteCounter >= (rxDataLen + 4))
	{
		UartRxStringLen = LPURxByteCounter;
		LPURxByteCounter = 0;
		rxDataLen = 0;
		LPURxBusy = 0;

		if (usedUart != LPUART1)
		{
			usedUart = LPUART1;
			CommInterface = IFC_UART;
			CommRxBuf = (uint8_t *) LPURxBuf;
			CommTxBuf = (uint8_t *) UartTxBuf;
			CommRxStringLen = (uint32_t *) &UartRxStringLen;
			CommRxBusy = (uint8_t *) &LPURxBusy;
			CommTxBusy = (uint8_t *) &UartTxBusy;
			CommTxAsyncBuf = (uint8_t *) UartTxAsyncBuf;
		}
	}

	UartRxTimeoutTimer = UART_TIMEOUT_MS;
}

/*-----------------------------------------------------------------------------
Name      :  serialTxCallback
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void serialTxCallback(USART_TypeDef *USARTx)
{
	if(UartTxByteCounter >= UartTxStringLen)
	{
		LL_USART_DisableIT_TXE_TXFNF (USARTx);
		LL_USART_EnableIT_TC(usedUart);
		return;
	}

	LL_USART_TransmitData8(USARTx, UartTxBuf[UartTxByteCounter]);
	UartTxByteCounter++;
}

/*-----------------------------------------------------------------------------
Name      :  serialTxCompleteCallback
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void serialTxCompleteCallback(USART_TypeDef *USARTx)
{
	if(USARTx == USART1)
	{
		LL_USART_DisableIT_TC(USARTx);
	}
	else if (USARTx == LPUART1)
	{
		LL_LPUART_DisableIT_TC(USARTx);
	}

	if ((usedUart == USART1) || (usedUart == LPUART1))
	{
		UartTxBusy++;
	}
}

/*-----------------------------------------------------------------------------
Name      :  serialTimer
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void serialTimer(void)
{
	if(UartRxTimeoutTimer)
	{
		UartRxTimeoutTimer--;
		if(UartRxTimeoutTimer == 0)
		{
			U1RxByteCounter = 0;
			U1RxBusy = 0;
			LPURxByteCounter = 0;
			LPURxBusy = 0;
		}
	}
}

/*-----------------------------------------------------------------------------
Name      :  startTxUart
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void startTxUart(uint32_t len)
{
	if(usedUart == USART1)
	{
		if(LL_LPUART_IsEnabled(LPUART1) == 1)
		{
			MX_LPUART1_UART_DeInit();
		}
	}
	else
	{
		if(LL_USART_IsEnabled(USART1) == 1)
		{
			MX_USART1_UART_DeInit();
		}
	}

	UartTxStringLen = len;
	UartTxByteCounter = 0;
	UartTxBusy = 1;
	LL_USART_EnableIT_TXE_TXFNF(usedUart);
}

/*-----------------------------------------------------------------------------
Name      :  wakeUpUart
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void wakeUpUart (void)
{
	if (usedUart == USART1)
	{
		MX_USART1_UART_Init();
	}
	else if (usedUart == LPUART1)
	{
		MX_LPUART1_UART_Init();
	}
	else
	{
		MX_USART1_UART_Init();
		MX_LPUART1_UART_Init();
	}

	updateUartSpeed();
	if (usedUart == USART1)
	{
		LL_USART_EnableIT_RXNE_RXFNE(USART1);
	}
	else if (usedUart == LPUART1)
	{
		LL_LPUART_EnableIT_RXNE_RXFNE(LPUART1);
	}
	else
	{
		LL_USART_EnableIT_RXNE_RXFNE(USART1);
		LL_LPUART_EnableIT_RXNE_RXFNE(LPUART1);
	}
}

/*-----------------------------------------------------------------------------
Name      :  sleepUart
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void sleepUart (void)
{
	if (usedUart == USART1)
	{
		MX_USART1_UART_DeInit();
	}
	else if (usedUart == LPUART1)
	{
		MX_LPUART1_UART_DeInit();
	}
	else
	{
		MX_USART1_UART_DeInit();
		MX_LPUART1_UART_DeInit();
	}
}

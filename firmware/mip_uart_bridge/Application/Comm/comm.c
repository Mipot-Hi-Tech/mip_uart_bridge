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
#include <string.h>

#include "comm.h"
#include "gpio.h"
#include "MainApp.h"
#include "serial.h"
#include "ipcc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t CommInterface;
uint8_t *CommTxBuf;
uint8_t *CommRxBuf;
uint32_t *CommRxStringLen;
uint8_t *CommRxBusy;
uint8_t *CommTxBusy;
uint8_t *CommTxAsyncBuf;

uint8_t uartBaudRateSel;
uint32_t shouldUpdateUartSpeed = 0;

static uint32_t SendMessageWithLen = 0;
static uint16_t SendMessageDelay = 0;

static uint32_t errCnt = 0;
static uint32_t msgCnt = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*-----------------------------------------------------------------------------
Name      :  calcChecksumSUM
Purpose   :  calculate checksum
Inputs    :  *buf: message received, bufLen: message length
Outputs   :  (uint8_t) checksum value
Return    :  <>
-----------------------------------------------------------------------------*/
static uint8_t calcChecksumSUM(volatile uint8_t *buf, uint32_t bufLen)
{
	uint8_t sum = 0;
	uint8_t i;

	for (i=0; i<bufLen; i++)
		sum += buf[i];

	sum = (sum ^ 0xFF) + 1;

	return (sum);
}

/*-----------------------------------------------------------------------------
Name      :  checksumCheckSUM
Purpose   :  verify checksum value
Inputs    :  *buf: message received, bufLen: message length
Outputs   :  (uint8_t) 1 - checksum correct | 0 - otherwise
Return    :  <>
-----------------------------------------------------------------------------*/
static uint8_t checksumCheckSUM(volatile uint8_t *buf, uint32_t bufLen)
{
	if (calcChecksumSUM (buf, bufLen) == 0)
	{
		return (1);
	}

	return (0);
}

/*-----------------------------------------------------------------------------
Name      :  commSetup
Purpose   :  serial communication initialization
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void commSetup(void)
{
	initUarts ();

	CommInterface = IFC_UART;
	CommRxBuf = (uint8_t *) U1RxBuf;
	CommTxBuf = (uint8_t *) UartTxBuf;
	CommRxStringLen = (uint32_t *) &UartRxStringLen;
	CommRxBusy = (uint8_t *) &U1RxBusy;
	CommTxBusy = (uint8_t *) &UartTxBusy;
	CommTxAsyncBuf = (uint8_t *) UartTxAsyncBuf;
}

/*-----------------------------------------------------------------------------
Name      :  commManager
Purpose   :  ipcc and serial communication management
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void commManager(void)
{
	if (*CommRxStringLen > 0)
	{
		if ((CommRxBuf[0] == CMD_HEADER_BYTE) && (CommRxBuf[1] == CMD_SET_LPM))
		{
			if (checksumCheckSUM (CommRxBuf, *CommRxStringLen))
			{
				//checksum ok
				if (CommRxBuf[2] == 1)
				{
					ipccServBuff[0] = (CommRxBuf[3] == 0) ? CM0_REQ_LPM : CM0_REQ_WUP;
					if (ipccServBuff[0] == CM0_REQ_LPM)
					{
						waitForSleep = 1;
						tmLpm = 5;
					}

					LL_EXTI_DisableIT_32_63(LL_EXTI_LINE_37);

					// Notify remote cpu of the on-going transaction
				    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_TX) != HAL_OK)
				    {
				    	Error_Handler();
				    }
				}
			}
		}
		else
		{
			if ((CommRxBuf[0] == CMD_HEADER_BYTE) && (CommRxBuf[1] == 0x30))
				HAL_NVIC_SystemReset();

			// Copy message to IPCC common buffer
			memcpy ((uint8_t *)ipccCommBuff, CommRxBuf, *CommRxStringLen);

			// Notify remote cpu of the on-going transaction
			if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_TX) != HAL_OK)
			{
				Error_Handler();
			}
		}

		memset (CommRxBuf, 0, COMM_BUF_MAX_LEN);
		*CommRxStringLen = 0;
	}

	if ((CM0_rx == CM0_TX_COMM) || (CM0_rx == CM0_RX_COMM))
	{
		if ((ipccCommBuff[0] == CMD_HEADER_BYTE) && (ipccCommBuff[2] < IPCC_COMM_SIZE - 4))
		{
			msgCnt++;
			if (checksumCheckSUM (ipccCommBuff, ipccCommBuff[2] + 4) == 0)
				errCnt++;

			SendMessageWithLen = ipccCommBuff[2] + 4;
			memcpy (CommTxBuf, (uint8_t *)ipccCommBuff, SendMessageWithLen);
		}

		if (CM0_rx == CM0_TX_COMM)
		{
		    // Notify remote cpu that transaction is completed
		    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_RX) != HAL_OK)
		    {
		    	Error_Handler();
		    }
		}
		CM0_rx = 0;

		memset ((uint8_t *) ipccCommBuff, 0, IPCC_COMM_SIZE);

		if (SendMessageWithLen > 0)
		{
			if(SendMessageDelay == 0)
			{
				if (CommInterface == IFC_UART)
				{
					startTxUart (SendMessageWithLen);
				}
				SendMessageWithLen = 0;
			}
		}
	}

	if (CM0_rx == CM0_RX_SERV)
	{
		CM0_rx = 0;

		switch (ipccServBuff[0])
		{
			case CM0_REQ_LPM:
				waitForSleep = 1;
				tmLpm = 5;
				break;

			case CM0_REQ_WUP:
				break;

			case CM0_REQ_UPD_BRATE:
				uartBaudRateSel = ipccServBuff[1];
				shouldUpdateUartSpeed = 1;
				break;
		}
		memset ((uint8_t *) ipccServBuff, 0, IPCC_SERV_SIZE);

	    // Notify remote cpu that transaction is completed
	    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_RX) != HAL_OK)
	    {
	    	Error_Handler();
	    }

	    if (shouldUpdateUartSpeed)
		{
			if(*CommTxBusy == 0)
			{
				shouldUpdateUartSpeed = 0;
				updateUartSpeed();
			}
		}
	}

	if (*CommTxBusy == 2)
	{
		(*CommTxBusy)++;

		ipccServBuff[0] = CM0_TX_END;

	    // Notify remote cpu of the on-going transaction
	    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_TX) != HAL_OK)
	    {
	    	Error_Handler();
	    }
	}
}

/*-----------------------------------------------------------------------------
Name      :  commTimer
Purpose   :  called every ms to advance local timer counters. Called after commManager
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void commTimer(void)
{
	serialTimer ();

	if (SendMessageDelay)
	{
		SendMessageDelay--;
	}
}

/*-----------------------------------------------------------------------------
Name      :  isCommIdle
Purpose   :  called every ms to advance local timer counters. Called after commManager
Inputs    :  <>
Outputs   :  (int) 1 - if no communication active | 0 - otherwise
Return    :  <>
-----------------------------------------------------------------------------*/
int isCommIdle(void)
{
	if((! *CommRxBusy) && (! *CommTxBusy) && (! *CommRxStringLen) && (!SendMessageWithLen))
	{
		return 1;
	}
	return 0;
}


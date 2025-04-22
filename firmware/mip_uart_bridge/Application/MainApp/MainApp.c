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
 *****************************************************************************/
#include "MainApp.h"
#include "MainTimer.h"
#include "comm.h"
#include "stm32_lpm_if.h"
#include "sys_conf.h"
#include "main.h"
#include "serial.h"
#include "sys_debug.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint8_t waitForSleep;
volatile uint32_t tmLpm;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*-----------------------------------------------------------------------------
Name      :  initMainApp
Purpose   :  <>
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void initMainApp (void)
{
	commSetup();

	waitForSleep = 0;
}

/*-----------------------------------------------------------------------------
Name      :  applicationProcess
Purpose   :  communication and low power management
Inputs    :  <>
Outputs   :  <>
Return    :  <>
-----------------------------------------------------------------------------*/
void applicationProcess(void)
{
	commManager();

	if (timerElapsed)
	{
		timerElapsed = 0;
		commTimer ();
	}

	if (waitForSleep == 1)
	{
		if (tmLpm == 0)
		{
			waitForSleep++;
		}
	}

	if ((waitForSleep == 2) && isCommIdle ())
	{
		/* Set MSI as wakeup clock source */
		LL_RCC_SetClkAfterWakeFromStop (LL_RCC_STOP_WAKEUPCLOCK_MSI);

		LL_RCC_HSE_Disable ();

		sleepUart();

		LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_28);
		LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_36);

		/* Enter STOP2 mode, WFI */
		PWR_EnterStopMode ();

		LL_RCC_HSE_Enable ();

		PWR_ExitStopMode ();

		/* Clear Stop2 flag of CPU1 */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP2);

		SystemClock_Config();

		wakeUpUart();

		waitForSleep = 0;
	}
}

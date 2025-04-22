/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32wlxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wlxx_ll_i2c.h"
#include "MainTimer.h"
#include "serial.h"
#include "gpio.h"
#include "ipcc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  mainTimerCallback();
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WLxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wlxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 Interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART1) == 1)
		U1RxCallback();

	if (LL_USART_IsActiveFlag_IDLE(USART1) == 1)
	{
		LL_USART_ClearFlag_IDLE(USART1);
		if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART1) == 1)
			U1RxCallback();
	}

	if(LL_USART_IsActiveFlag_ORE(USART1) == 1)
	{
		LL_USART_ClearFlag_ORE(USART1);
	}

	if(LL_USART_IsActiveFlag_TXE_TXFNF(USART1) == 1)
	{
		if (LL_USART_IsEnabledIT_TXE_TXFNF (USART1) == 1)
		{
			serialTxCallback(USART1);
		}
	}

	if (LL_USART_IsActiveFlag_TC(USART1) == 1)
	{
		if (LL_USART_IsEnabledIT_TC (USART1) == 1)
		{
			serialTxCompleteCallback (USART1);
		}
		else
		{
			LL_USART_ClearFlag_TC (USART1);
		}
	}

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 Interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */
	if(LL_LPUART_IsActiveFlag_RXNE_RXFNE(LPUART1) == 1)
	{
		LPURxCallback();
	}
	if(LL_LPUART_IsActiveFlag_IDLE(LPUART1) == 1)
	{
		LL_LPUART_ClearFlag_IDLE(LPUART1);
		if(LL_LPUART_IsActiveFlag_RXNE_RXFNE(LPUART1) == 1)
		{
			LPURxCallback();
		}
	}
	if(LL_LPUART_IsActiveFlag_ORE(LPUART1) == 1)
	{
		LL_LPUART_ClearFlag_ORE(LPUART1);
	}
	if(LL_LPUART_IsActiveFlag_TXE_TXFNF(LPUART1) == 1)
	{
		if (LL_LPUART_IsEnabledIT_TXE_TXFNF (LPUART1) == 1)
		{
			serialTxCallback(LPUART1);
		}
	}
	if (LL_LPUART_IsActiveFlag_TC(LPUART1) == 1)
	{
		if (LL_LPUART_IsEnabledIT_TC (LPUART1) == 1)
		{
			serialTxCompleteCallback(LPUART1);
		}
		else
		{
			LL_LPUART_ClearFlag_TC (LPUART1);
		}
	}
  /* USER CODE END LPUART1_IRQn 0 */
  /* USER CODE BEGIN LPUART1_IRQn 1 */

  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief  This function handles PVD interrupt request.
  * @param  None
  * @retval None
  */
void PVD_PVM_IRQHandler(void)
{
  /* Loop inside the handler to prevent the Cortex from using the Flash,
     allowing the flash interface to finish any ongoing transfer. */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET)
  {
  }
}

/**
  * @brief This function handles IPCC RX Occupied Interrupt.
  */
void IPCC_C1_RX_IRQHandler(void)
{
  /* USER CODE BEGIN IPCC_C1_RX_IRQn 0 */

  /* USER CODE END IPCC_C1_RX_IRQn 0 */
  HAL_IPCC_RX_IRQHandler(&hipcc);
  /* USER CODE BEGIN IPCC_C1_RX_IRQn 1 */

  /* USER CODE END IPCC_C1_RX_IRQn 1 */
}

/**
  * @brief This function handles IPCC TX Free Interrupt.
  */
void IPCC_C1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN IPCC_C1_TX_IRQn 0 */

  /* USER CODE END IPCC_C1_TX_IRQn 0 */
  HAL_IPCC_TX_IRQHandler(&hipcc);
  /* USER CODE BEGIN IPCC_C1_TX_IRQn 1 */

  /* USER CODE END IPCC_C1_TX_IRQn 1 */
}


void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_7);
}
/* USER CODE END 1 */

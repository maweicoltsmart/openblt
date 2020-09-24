/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "CH57x_common.h"
#include "core_cm0.h"
#include "vector.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
  printf("%s, %d\r\n", __func__, __LINE__);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
/*void HardFault_Handler(void)
{

}*/
void ETH_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_ETH_ADDR);
}

void GPIO_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_GPIO_ADDR);
}

void SLAVE_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_SLAVE_ADDR);
}

void SPI0_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_SPI0_ADDR);
}

void BB_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_BLEB_ADDR);
}

void LLE_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_BLEL_ADDR);
}

void USB_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_USB_ADDR);
}

void TMR1_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_TIMER1_ADDR);
}

void TMR2_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_TIMER2_ADDR);
}

void RTC_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_RTC_ADDR);
}

void ADC_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_ADC_ADDR);
}

void SPI1_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_SPI1_ADDR);
}

void LED_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_LED_ADDR);
}

void TMR3_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_TIMER3_ADDR);
}

void UART2_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_UART2_ADDR);
}

void UART3_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_UART3_ADDR);
}

void WDT_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_WWDOG_BAT_ADDR);
}

/*******************************************************************************
* Function Name  : TMR0_IRQHandler
* Description    : IRQ中断服务函数，10ms中断一次
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TMR0_IRQHandler(void) /* 定时器0中断 */
{
  GOTO_APP(APP_VECTOR_TIMER0_ADDR);
}
/*******************************************************************************
* Function Name  : UART0_IRQHandler
* Description    : IRQ中断服务函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART0_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_UART0_ADDR);
}
/*******************************************************************************
* Function Name  : UART1_IRQHandler
* Description    : IRQ中断服务函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART1_IRQHandler(void)
{
  GOTO_APP(APP_VECTOR_UART1_ADDR);
}
/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

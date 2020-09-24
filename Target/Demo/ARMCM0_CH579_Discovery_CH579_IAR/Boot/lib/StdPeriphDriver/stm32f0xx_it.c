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
void ETH_IRQHandler(void) @".ethvec"
{
}

void GPIO_IRQHandler(void) @".gpiovec"
{
}

void SLAVE_IRQHandler(void) @".slavevec"
{
}

void SPI0_IRQHandler(void) @".spi0vec"
{
}

void BB_IRQHandler(void) @".blebvec"
{
}

void LLE_IRQHandler(void) @".blelvec"
{
}

void USB_IRQHandler(void) @".usbvec"
{
}

void TMR1_IRQHandler(void) @".timer1vec"
{
}

void TMR2_IRQHandler(void) @".timer2vec"
{
}

void RTC_IRQHandler(void) @".rtcvec"
{
}

void ADC_IRQHandler(void) @".adcvec"
{
}

void SPI1_IRQHandler(void) @".spi1vec"
{
}

void LED_IRQHandler(void) @".ledvec"
{
}

void TMR3_IRQHandler(void) @".timer3vec"
{
}

void UART2_IRQHandler(void) @".uart2vec"
{
}

void UART3_IRQHandler(void) @".uart3vec"
{
}

void WDT_IRQHandler(void) @".wwdogvec"
{
}

/*******************************************************************************
* Function Name  : TMR0_IRQHandler
* Description    : IRQ中断服务函数，10ms中断一次
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TMR0_IRQHandler(void) @".timer0vec" /* 定时器0中断 */
{
}
/*******************************************************************************
* Function Name  : UART0_IRQHandler
* Description    : IRQ中断服务函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART0_IRQHandler(void) @".uart0vec"
{
}
/*******************************************************************************
* Function Name  : UART1_IRQHandler
* Description    : IRQ中断服务函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART1_IRQHandler(void) @".uart1vec"
{
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

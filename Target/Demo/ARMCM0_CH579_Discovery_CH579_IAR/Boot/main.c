/************************************************************************************//**
* \file         Demo/ARMCM0_STM32F0_Discovery_STM32F051_IAR/Boot/main.c
* \brief        Bootloader application source file.
* \ingroup      Boot_ARMCM0_STM32F0_Discovery_STM32F051_IAR
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2016  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It 
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#include "CH57x_common.h"
#include "vector.h"
#include "stdio.h"

/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void Init(void);
//static void SystemClock_Config(void);


/************************************************************************************//**
** \brief     This is the entry point for the bootloader application and is called
**            by the reset interrupt vector after the C-startup routines executed.
** \return    none.
**
****************************************************************************************/
void main(void)
{
  /* Initialize the microcontroller. */
  Init();
  printf("Boot Power on\r\n");
  /*__set_PSP(*(volatile unsigned int*) APP_VECTOR_TABLE_START_ADDR);
  __set_CONTROL(0);
  __set_MSP(*(volatile unsigned int*) APP_VECTOR_TABLE_START_ADDR);
  GOTO_APP(APP_VECTOR_RESET_ADDR);*/
  /* Initialize the bootloader. */
  BootInit();

  /* Start the infinite program loop. */
  while (1)
  {
    /* Run the bootloader task. */
    BootTask();
  }
} /*** end of main ***/

/************************************************************************************//**
** \brief     Initializes the microcontroller.
** \return    none.
**
****************************************************************************************/
static void Init(void)
{
  GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_20mA);
  GPIOA_ResetBits(GPIO_Pin_15);// STATUS 拉低, 
  PWR_UnitModCfg(ENABLE, UNIT_SYS_PLL); /* PLL上电 */
  DelayMs(3);
  SetSysClock(CLK_SOURCE_HSE_32MHz); /* 外部晶振 PLL 输出32MHz */
  HClk32M_Select(Clk32M_HSE);
  /* HAL library initialization */
  //HAL_Init();
  /* 配置串口1：先配置IO口模式，再配置串口 */
  GPIOA_SetBits(GPIO_Pin_9);
  GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);			// RXD-配置上拉输入
  GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);		// TXD-配置推挽输出，注意先让IO口输出高电平
  UART1_DefInit();  // 默认115200，8N1

  UART1_ByteTrigCfg( UART_1BYTE_TRIG );  // 串口字节触发中断配置
  //UART1_INTCfg( ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT );
  //NVIC_SetPriority(UART1_IRQn, 3);
  //NVIC_EnableIRQ( UART1_IRQn );
  /* configure system clock */
  //SystemClock_Config();
} /*** end of Init ***/

#if 0
/************************************************************************************//**
** \brief     System Clock Configuration. This code was created by CubeMX and configures
**            the system clock to match the configuration in the bootloader's
**            configuration (blt_conf.h), specifically the macros:
**            BOOT_CPU_SYSTEM_SPEED_KHZ and BOOT_CPU_XTAL_SPEED_KHZ.
**            Note that the Lower Layer drivers were selected in CubeMX for the RCC
**            subsystem.
** \return    none.
**
****************************************************************************************/
static void SystemClock_Config(void)
{
  /* Set flash latency. */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  /* Verify flash latency setting. */
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
    /* Error setting flash latency. */
    ASSERT_RT(BLT_FALSE);
  }
  
  /* Enable the HSE clock and bypass as the MCO signal from the ST-link is used. */
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    ; 
  }
  /* Configure and enable the PLL. */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();
  /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    ;  
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
    ;
  }
  /* Update the system clock speed setting. */
  LL_SetSystemCoreClock(BOOT_CPU_SYSTEM_SPEED_KHZ * 1000u);
} /*** end of SystemClock_Config ***/


/************************************************************************************//**
** \brief     Initializes the Global MSP. This function is called from HAL_Init()
**            function to perform system level initialization (GPIOs, clock, DMA,
**            interrupt).
** \return    none.
**
****************************************************************************************/
void HAL_MspInit(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* SYSCFG clock enable. */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  /* GPIO ports clock enable. */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

#if (BOOT_COM_RS232_ENABLE > 0)
  /* UART clock enable. */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
#endif

  /* Configure GPIO pin for the LED. */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);

  /* Configure GPIO pin for (optional) backdoor entry input. */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if (BOOT_COM_RS232_ENABLE > 0)
  /* UART TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
} /*** end of HAL_MspInit ***/


/************************************************************************************//**
** \brief     DeInitializes the Global MSP. This function is called from HAL_DeInit()
**            function to perform system level de-initialization (GPIOs, clock, DMA,
**            interrupt).
** \return    none.
**
****************************************************************************************/
void HAL_MspDeInit(void)
{
  /* Reset the RCC clock configuration to the default reset state. */
  LL_RCC_DeInit();

  /* Reset GPIO pin for the LED to turn it off. */
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);

  /* Deinit used GPIOs. */
  LL_GPIO_DeInit(GPIOC);
  LL_GPIO_DeInit(GPIOA);

#if (BOOT_COM_RS232_ENABLE > 0)
  /* UART clock disable. */
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
#endif

  /* GPIO ports clock disable. */
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* SYSCFG clock disable. */
  LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
} /*** end of HAL_MspDeInit ***/
#endif

/*********************************** end of main.c *************************************/

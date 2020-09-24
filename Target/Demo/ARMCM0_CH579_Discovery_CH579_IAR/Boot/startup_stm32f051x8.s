;******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
;* File Name          : startup_stm32f051x8.s
;* Author             : MCD Application Team
;* Description        : STM32F051x4/STM32F051x6/STM32F051x8 devices vector table 
;*                      for EWARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == __iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR 
;*                        address,
;*                      - Branches to main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the Cortex-M0 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;*******************************************************************************
;*
;* Redistribution and use in source and binary forms, with or without modification,
;* are permitted provided that the following conditions are met:
;*   1. Redistributions of source code must retain the above copyright notice,
;*      this list of conditions and the following disclaimer.
;*   2. Redistributions in binary form must reproduce the above copyright notice,
;*      this list of conditions and the following disclaimer in the documentation
;*      and/or other materials provided with the distribution.
;*   3. Neither the name of STMicroelectronics nor the names of its contributors
;*      may be used to endorse or promote products derived from this software
;*      without specific prior written permission.
;*
;* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
;* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;*
;*******************************************************************************
;
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler                  ; Reset Handler

        DCD     NMI_Handler                    ; NMI Handler
        DCD     HardFault_Handler              ; Hard Fault Handler
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     SVC_Handler                    ; SVCall Handler
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     PendSV_Handler                 ; PendSV Handler
        DCD     SysTick_Handler                ; SysTick Handler

        ; External Interrupts
        DCD     TMR0_IRQHandler           ;  0:  TMR0
        DCD     GPIO_IRQHandler           ;  1:  GPIO
        DCD     SLAVE_IRQHandler          ;  2:  SLAVE
        DCD     SPI0_IRQHandler           ;  3:  SPI0
        DCD     BB_IRQHandler             ;  4:  BB
        DCD     LLE_IRQHandler            ;  5:  LLE
        DCD     USB_IRQHandler            ;  6:  USB
        DCD     ETH_IRQHandler            ;  7:  ETH
        DCD     TMR1_IRQHandler           ;  8:  TMR1
        DCD     TMR2_IRQHandler           ;  9:  TMR2
        DCD     UART0_IRQHandler          ; 10:  UART0
        DCD     UART1_IRQHandler          ; 11:  UART1
        DCD     RTC_IRQHandler            ; 12:  RTC
        DCD     ADC_IRQHandler            ; 13:  ADC
        DCD     SPI1_IRQHandler           ; 14:  SPI1
        DCD     LED_IRQHandler            ; 15:  LED
        DCD     TMR3_IRQHandler           ; 16:  TMR3 
        DCD     UART2_IRQHandler          ; 17:  UART2
        DCD     UART3_IRQHandler          ; 18:  UART3
        DCD     WDT_IRQHandler            ; 19:  WDT
        
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:NOROOT:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0
        
        PUBWEAK NMI_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SysTick_Handler
        B SysTick_Handler
        
        
        PUBWEAK TMR0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TMR0_IRQHandler
        B TMR0_IRQHandler

        PUBWEAK GPIO_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
GPIO_IRQHandler
        B GPIO_IRQHandler

        PUBWEAK SLAVE_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SLAVE_IRQHandler
        B SLAVE_IRQHandler

        PUBWEAK SPI0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI0_IRQHandler
        B SPI0_IRQHandler

        PUBWEAK BB_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
BB_IRQHandler
        B BB_IRQHandler

        PUBWEAK LLE_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LLE_IRQHandler
        B LLE_IRQHandler

        PUBWEAK USB_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USB_IRQHandler
        B USB_IRQHandler

        PUBWEAK ETH_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
ETH_IRQHandler
        B ETH_IRQHandler

        PUBWEAK TMR1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TMR1_IRQHandler
        B TMR1_IRQHandler

        PUBWEAK TMR2_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TMR2_IRQHandler
        B TMR2_IRQHandler

        PUBWEAK UART0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART0_IRQHandler
        B UART0_IRQHandler

        PUBWEAK UART1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART1_IRQHandler
        B UART1_IRQHandler

        PUBWEAK RTC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
RTC_IRQHandler
        B RTC_IRQHandler

        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
ADC_IRQHandler
        B ADC_IRQHandler

        PUBWEAK SPI1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI1_IRQHandler
        B SPI1_IRQHandler

        PUBWEAK LED_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LED_IRQHandler
        B LED_IRQHandler

        PUBWEAK TMR3_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TMR3_IRQHandler
        B TMR3_IRQHandler

        PUBWEAK UART2_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART2_IRQHandler
        B UART2_IRQHandler

        PUBWEAK UART3_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART3_IRQHandler
        B UART3_IRQHandler

        PUBWEAK WDT_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
WDT_IRQHandler
        B WDT_IRQHandler                  

        END
;************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE*****

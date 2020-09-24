#ifndef __APP_VECTOR_TABLE__
#define __APP_VECTOR_TABLE__

#define APP_VECTOR_TABLE_START_ADDR                      (0x00000000 + 16 * 1024)
#define APP_VECTOR_RESET_ADDR                            (0x00000004 + 16 * 1024)
#define APP_VECTOR_SYSTICK_ADDR                          (0x0000003C + 16 * 1024)
#define APP_VECTOR_TIMER0_ADDR                           (0x00000040 + 16 * 1024)
#define APP_VECTOR_GPIO_ADDR                             (0x00000044 + 16 * 1024)
#define APP_VECTOR_SLAVE_ADDR                            (0x00000048 + 16 * 1024)
#define APP_VECTOR_SPI0_ADDR                             (0x0000004C + 16 * 1024)
#define APP_VECTOR_BLEL_ADDR                             (0x00000050 + 16 * 1024)
#define APP_VECTOR_BLEB_ADDR                             (0x00000054 + 16 * 1024)
#define APP_VECTOR_USB_ADDR                              (0x00000058 + 16 * 1024)
#define APP_VECTOR_ETH_ADDR                              (0x0000005C + 16 * 1024)
#define APP_VECTOR_TIMER1_ADDR                           (0x00000060 + 16 * 1024)
#define APP_VECTOR_TIMER2_ADDR                           (0x00000064 + 16 * 1024)
#define APP_VECTOR_UART0_ADDR                            (0x00000068 + 16 * 1024)
#define APP_VECTOR_UART1_ADDR                            (0x0000006C + 16 * 1024)
#define APP_VECTOR_RTC_ADDR                              (0x00000070 + 16 * 1024)
#define APP_VECTOR_ADC_ADDR                              (0x00000074 + 16 * 1024)
#define APP_VECTOR_SPI1_ADDR                             (0x00000078 + 16 * 1024)
#define APP_VECTOR_LED_ADDR                              (0x0000007C + 16 * 1024)
#define APP_VECTOR_TIMER3_ADDR                           (0x00000080 + 16 * 1024)
#define APP_VECTOR_UART2_ADDR                            (0x00000084 + 16 * 1024)
#define APP_VECTOR_UART3_ADDR                            (0x00000088 + 16 * 1024)
#define APP_VECTOR_WWDOG_BAT_ADDR                        (0x0000008C + 16 * 1024)

#define GOTO_APP(ADDR)                                   ((void (*)(void))(*(uint32_t *)(ADDR)))()
#endif
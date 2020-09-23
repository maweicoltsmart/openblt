/********************************** (C) COPYRIGHT *******************************
* File Name          : CH57x_uart1.c
* Author             : WCH
* Version            : V1.0
* Date               : 2018/12/15
* Description 
*******************************************************************************/
#include "stdio.h"
#include "CH57x_common.h"
//#include "pragma.h"

/*******************************************************************************
* Function Name  : UART1_DefInit
* Description    : 串口默认初始化配置
* Input          : None
* Return         : None
*******************************************************************************/
#define RB_LCR_WORD_SZ_5                0x00
#define RB_LCR_WORD_SZ_6                0x01
#define RB_LCR_WORD_SZ_7                0x02
#define RB_LCR_WORD_SZ_8                0x03

#define RB_LCR_STOP_BIT_1               0x00
#define RB_LCR_STOP_BIT_2               0x04

#define RB_LCR_PAR_EN                   0x08
#define RB_LCR_PAR_MOD_ODD              0x00
#define RB_LCR_PAR_MOD_EVEN             0x10
#define RB_LCR_PAR_MOD_MARK             0x20
#define RB_LCR_PAR_MOD_SPACE            0x30

void UART1_DefInit( void )
{
    uint8_t cfg = 0;
    UART1_BaudRateCfg( 115200 );
    R8_UART1_FCR = (2<<6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;		// FIFO打开，触发点4字节
    cfg |= RB_LCR_WORD_SZ_8;
    cfg |= RB_LCR_STOP_BIT_1;
    cfg |= 0x00;

    R8_UART1_LCR = cfg;	
    R8_UART1_IER = RB_IER_TXD_EN;
    R8_UART1_DIV = 1;	
}

/*******************************************************************************
* Function Name  : UART1_BaudRateCfg
* Description    : 串口波特率配置
* Input          : 
* Return         : 
*******************************************************************************/
void UART1_BaudRateCfg( UINT32 baudrate )
{
    UINT32	x;

    x = 10 * FREQ_SYS / 8 / baudrate;
    x = ( x + 5 ) / 10;
    R16_UART1_DL = (UINT16)x;
}

/*******************************************************************************
* Function Name  : UART1_ByteTrigCfg
* Description    : 串口字节触发中断配置
* Input          : b: 触发字节数
                    refer to UARTByteTRIGTypeDef
* Return         : 
*******************************************************************************/
void UART1_ByteTrigCfg( UARTByteTRIGTypeDef b )
{
    R8_UART1_FCR = (R8_UART1_FCR&~RB_FCR_FIFO_TRIG)|(b<<6);
}

/*******************************************************************************
* Function Name  : UART1_INTCfg
* Description    : 串口中断配置
* Input          : s:  中断控制状态
					ENABLE  - 使能相应中断    
					DISABLE - 关闭相应中断
				   i:  中断类型
					RB_IER_MODEM_CHG  - 调制解调器输入状态变化中断使能位（仅 UART0 支持）
					RB_IER_LINE_STAT  - 接收线路状态中断
					RB_IER_THR_EMPTY  - 发送保持寄存器空中断
					RB_IER_RECV_RDY   - 接收数据中断
* Return         : None
*******************************************************************************/
void UART1_INTCfg( UINT8 s,  UINT8 i )
{
    if( s )
    {
        R8_UART1_IER |= i;
        R8_UART1_MCR |= RB_MCR_INT_OE;
    }
    else
    {
        R8_UART1_IER &= ~i;
    }
}

/*******************************************************************************
* Function Name  : UART1_Reset
* Description    : 串口软件复位
* Input          : None
* Return         : None
*******************************************************************************/
void UART1_Reset( void )
{
    R8_UART1_IER = RB_IER_RESET;
}

/*******************************************************************************
* Function Name  : UART1_SendString
* Description    : 串口多字节发送
* Input          : buf - 待发送的数据内容首地址
                     l - 待发送的数据长度
* Return         : None
*******************************************************************************/
void UART1_SendString( PUINT8 buf, UINT16 l )
{
    UINT16 len = l;

    while(len)
    {
        if(R8_UART1_TFC != UART_FIFO_SIZE)
        {
            R8_UART1_THR = *buf++;
            len--;
        }		
    }
}

/*******************************************************************************
* Function Name  : fputc
* Description    : surport for printf
* Input          : ch - 待发送的数据内容首地址
                     f - 文件句柄
* Return         : ch
*******************************************************************************/
//重定向c库函数printf到USART1
int fputc(int ch, FILE *f)
{
    //return (ch);
    R8_UART1_THR = (uint8_t) ch;
    while(R8_UART1_TFC == UART_FIFO_SIZE);
    return (ch);
}


/*******************************************************************************
* Function Name  : UART1_RecvString
* Description    : 串口读取多字节
* Input          : buf - 读取数据存放缓存区首地址
* Return         : 读取数据长度
*******************************************************************************/
UINT16 UART1_RecvString( PUINT8 buf )
{
    UINT16 len = 0;

    while( R8_UART1_RFC )
    {
        *buf++ = R8_UART1_RBR;
        len ++;
    }

    return (len);
}



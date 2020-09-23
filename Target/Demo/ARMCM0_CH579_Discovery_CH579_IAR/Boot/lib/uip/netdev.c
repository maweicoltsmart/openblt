/*
 * Copyright (c) 2001, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: netdev.c,v 1.8 2006/06/07 08:39:58 adam Exp $
 */


/*---------------------------------------------------------------------------*/
#include "uip.h"
#include "uip_arp.h"
#include "boot.h"
#include "CH57x_common.h"
#include "ETH.h"

/*---------------------------------------------------------------------------*/
#define NETDEV_LINKUP_TIMEOUT_MS          (5000)

#define NETDEV_DEFAULT_MACADDR0           (0x08)
#define NETDEV_DEFAULT_MACADDR1           (0x00)
#define NETDEV_DEFAULT_MACADDR2           (0x27)
#define NETDEV_DEFAULT_MACADDR3           (0x69)
#define NETDEV_DEFAULT_MACADDR4           (0x5B)
#define NETDEV_DEFAULT_MACADDR5           (0x45)


/*---------------------------------------------------------------------------*/
static struct uip_eth_addr macAddress;
static UINT8 ethrxbuffer[ETH_MAX_PACKET_SIZE];
static UINT8 ethtxbuffer[ETH_MAX_PACKET_SIZE];

UINT16 ReadPHYReg(UINT8 reg_add)
{
    UINT8 reg_op=0;
    UINT16 read_reg_val = 0xffff;

    reg_op = reg_add&RB_ETH_MIREGADR_MIRDL;
    R8_ETH_MIREGADR = RB_ETH_MIREGADR_MIIWR|reg_op;
    read_reg_val = R16_ETH_MIRD;

    return read_reg_val;
}

void LED_CONN_ON(UINT8 on)
{
    on?GPIOB_ResetBits(GPIO_Pin_4):GPIOB_SetBits(GPIO_Pin_4);
}

void LED_DATA_ON(UINT8 on)
{
    on?GPIOB_SetBits(GPIO_Pin_7):GPIOB_ResetBits(GPIO_Pin_7);
}

uint32_t TimerGetElapsedTime( uint32_t savedTime )
{
    volatile uint32_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    /*if( savedTime == 0 )
    {
        return 0;
    }*/

    elapsedTime = TimerGet();

    if( elapsedTime < savedTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - savedTime ) );
    }
    else
    {
        return( elapsedTime - savedTime );
    }
}

/*---------------------------------------------------------------------------*/
void netdev_init(void)
{
  /* enable and reset the ethernet controller. */
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);
  //SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);
  R8_ETH_ECON1 &= ~RB_ETH_ECON1_RXEN;                                           //接收使能
  R8_ETH_ECON1 |= (RB_ETH_ECON1_TXRST|RB_ETH_ECON1_RXRST);                      //收发模块复位
  R8_ETH_ECON1 &= ~(RB_ETH_ECON1_TXRST|RB_ETH_ECON1_RXRST);
}


/*---------------------------------------------------------------------------*/
void netdev_init_mac(void)
{
  //unsigned long ulUser0, ulUser1;
  blt_int32u ulTemp;
  blt_int32u ulLinkTimeOut;

  /* enable port F for ethernet LEDs.
   *  LED0        Bit 3   Output
   *  LED1        Bit 2   Output
   */
  UINT8 TestMAC[6] = {0x84,0xc2,0xe4,0x02,0x03,0x04};
  macAddress.addr[5] = TestMAC[5];                                                 //MAC赋值
  macAddress.addr[4] = TestMAC[4];
  macAddress.addr[3] = TestMAC[3];
  macAddress.addr[2] = TestMAC[2];
  macAddress.addr[1] = TestMAC[1];
  macAddress.addr[0] = TestMAC[0];
  GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_20mA);
  GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_20mA);
  LED_CONN_ON(0);
  LED_DATA_ON(0);
  /* intialize the ethernet controller and disable all ethernet controller
   * interrupt sources.
   */
  R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
  R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
  R8_SLP_CLK_OFF1 &= ~RB_SLP_CLK_ETH;
  R8_SLP_POWER_CTRL &= ~RB_SLP_ETH_PWR_DN;
  R8_SAFE_ACCESS_SIG = 0;

  R8_ETH_EIE = 0;
  R8_ETH_EIE |= RB_ETH_EIE_INTIE |
                RB_ETH_EIE_RXIE  |
                RB_ETH_EIE_LINKIE|
                RB_ETH_EIE_TXIE  ;
                //RB_ETH_EIE_TXERIE|
                //RB_ETH_EIE_RXERIE;                                            //开启所有中断


  R8_ETH_EIE |= RB_ETH_EIE_R_EN50;                                              //开启50欧上拉
  R8_ETH_EIR = 0xff;                                                            //清除中断标志
  R8_ETH_ESTAT |= RB_ETH_ESTAT_INT | RB_ETH_ESTAT_BUFER;                        //清除状态

  R8_ETH_ECON1 |= (RB_ETH_ECON1_TXRST|RB_ETH_ECON1_RXRST);                      //收发模块复位
  R8_ETH_ECON1 &= ~(RB_ETH_ECON1_TXRST|RB_ETH_ECON1_RXRST);
  //接收使能在link中断里写

  //过滤模式，接收包类型
  R8_ETH_ERXFCON = 0;  //
  R8_ETH_ERXFCON &= ~RB_ETH_ERXFCON_ANDOR;
  R8_ETH_ERXFCON &= ~RB_ETH_ERXFCON_MPEN;
  R8_ETH_ERXFCON &= ~RB_ETH_ERXFCON_HTEN;
  R8_ETH_ERXFCON |= RB_ETH_ERXFCON_UCEN |                                       //目标地址匹配将被接收
                    RB_ETH_ERXFCON_CRCEN|                                       //CRC校验正确将被接收
                    RB_ETH_ERXFCON_MCEN  |                                      //组播包匹配将被接收
                    RB_ETH_ERXFCON_BCEN  ;                                      //广播包将被接收

  //过滤模式，限制包类型

  R8_ETH_MACON1 |= RB_ETH_MACON1_MARXEN;                                        //MAC接收使能
  R8_ETH_MACON1 &= ~RB_ETH_MACON1_PASSALL;                                      //过滤控制帧
  R8_ETH_MACON1 |= RB_ETH_MACON1_RXPAUS;                                        //接收暂停帧
  R8_ETH_MACON1 |= RB_ETH_MACON1_TXPAUS;                                        //发送pause帧使能
  R16_ETH_EPAUS = 100;

  R8_ETH_MACON2 |= PADCFG_AUTO_3;                                               //所有短包自动填充到60
  //R8_ETH_MACON2 |= PADCFG_NO_ACT_0;

  R8_ETH_MACON2 |= RB_ETH_MACON2_TXCRCEN;                                       //硬件填充CRC
  R8_ETH_MACON2 &= ~RB_ETH_MACON2_HFRMEN;                                       //不接收巨型帧


  R8_ETH_MACON2 |= RB_ETH_MACON2_FULDPX;                                        //全双工
  //R8_ETH_MACON2 &= ~RB_ETH_MACON2_FULDPX;                                     //半双工

  R16_ETH_MAMXFL = MAC_MAX_LEN; 
  R16_PIN_ANALOG_IE |= RB_PIN_ETH_IE;
  R16_ETH_ERXST = (UINT16)ethrxbuffer;                                          // 更新当前接收缓存
  R8_ETH_ECON1 |= RB_ETH_ECON1_RXEN;                                            //接收使能
  //NVIC_EnableIRQ(ETH_IRQn);
  /* wait for the link to become active. */
  ulTemp = R8_ETH_EIR;
  R8_ETH_EIR = RB_ETH_EIR_LINKIF;
  UINT16 phy_reg;
  phy_reg = ReadPHYReg(PHY_BMSR);                                               //读取PHY状态寄存器
  ulLinkTimeOut = TimerGet();
  //printf("%s, %d, %d, %d\r\n", __func__, __LINE__, ulTemp, phy_reg);
  while(((ulTemp & RB_ETH_EIR_LINKIF) == 0) || (phy_reg&0x04 == 0))
  {
    ulTemp = R8_ETH_EIR;
    R8_ETH_EIR = RB_ETH_EIR_LINKIF;
    phy_reg = ReadPHYReg(PHY_BMSR);                                               //读取PHY状态寄存器
    //printf("%s, %d, %d, %d\r\n", __func__, __LINE__, ulTemp, phy_reg);
    /* check for timeout so that the software program can still start if the 
     * ethernet cable is not connected.
     */
    if (TimerGetElapsedTime(ulLinkTimeOut) >= NETDEV_LINKUP_TIMEOUT_MS)
    {
      break;
    }
    /* Service the watchdog. */
    CopService();
  }
  LED_CONN_ON(1);
  /* set the default MAC address */
  R8_ETH_MAADR1 = macAddress.addr[5];                                                 //MAC赋值
  R8_ETH_MAADR2 = macAddress.addr[4];
  R8_ETH_MAADR3 = macAddress.addr[3];
  R8_ETH_MAADR4 = macAddress.addr[2];
  R8_ETH_MAADR5 = macAddress.addr[1];
  R8_ETH_MAADR6 = macAddress.addr[0];
  /* the LM3S eval kit should have a MAC address pre-propgrammed in flash by the 
   * manufacturer. try to use this one, otherwise use the default values.
   */
  /*
  FlashUserGet(&ulUser0, &ulUser1);
  if ( (ulUser0 != 0xffffffff) && (ulUser1 != 0xffffffff) )
  {
    macAddress.addr[0] = ((ulUser0 >>  0) & 0xff);
    macAddress.addr[1] = ((ulUser0 >>  8) & 0xff);
    macAddress.addr[2] = ((ulUser0 >> 16) & 0xff);
    macAddress.addr[3] = ((ulUser1 >>  0) & 0xff);
    macAddress.addr[4] = ((ulUser1 >>  8) & 0xff);
    macAddress.addr[5] = ((ulUser1 >> 16) & 0xff);
  }
  EthernetMACAddrSet(ETH_BASE, &macAddress.addr[0]);
*/
  uip_setethaddr(macAddress);
}

static uint8_t eth_data_rx_tx_flag = 0;
static blt_int32u eth_data_led_on_timeout = 0;
void eth_data_led(void)
{
  if(eth_data_rx_tx_flag)
  {
    LED_DATA_ON(1);
    if(TimerGetElapsedTime(eth_data_led_on_timeout) >= 100)
    {
      LED_DATA_ON(0);
      eth_data_rx_tx_flag = 0;
    }
  }
}
/*---------------------------------------------------------------------------*/
void netdev_get_mac(unsigned char * mac_addr)
{
  mac_addr[0] = macAddress.addr[0];
  mac_addr[1] = macAddress.addr[1];
  mac_addr[2] = macAddress.addr[2];
  mac_addr[3] = macAddress.addr[3];
  mac_addr[4] = macAddress.addr[4];
  mac_addr[5] = macAddress.addr[5];
}


/*---------------------------------------------------------------------------*/
unsigned int netdev_read(void)
{
  blt_int32u ulTemp;

  /* read and Clear the interrupt flag. */
  ulTemp = R8_ETH_EIR;
  R8_ETH_EIR = RB_ETH_EIR_RXIF;
  //EthernetIntClear(ETH_BASE, ulTemp);

  /* check to see if an RX Interrupt has occured. */
  if(ulTemp & RB_ETH_EIR_RXIF)
  {
    if(!eth_data_rx_tx_flag)
    {
      eth_data_rx_tx_flag = 1;
      eth_data_led_on_timeout = TimerGet();
    }
    //printf("eth recv\r\n");
    int len = 0;
    len = R16_ETH_ERXLN;
    uip_len = len;
    //printf("eth recv: %d bytes\r\n", len);
    memcpy(uip_buf, ethrxbuffer, R16_ETH_ERXLN);
    R16_ETH_ERXST = (UINT16)ethrxbuffer;
    return len;//EthernetPacketGetNonBlocking(ETH_BASE, uip_buf, sizeof(uip_buf));
  }
  return 0;
}

void uip_log(char *msg)
{
  printf("%s\r\n", msg);
}
/*---------------------------------------------------------------------------*/
void netdev_send(void)
{
  UINT8 eth_irq_flag;
  if(!eth_data_rx_tx_flag)
  {
    eth_data_rx_tx_flag = 1;
    eth_data_led_on_timeout = TimerGet();
  }
  memcpy(ethtxbuffer, uip_buf, uip_len);
  //printf("eth send %d bytes\r\n", uip_len);
  R8_ETH_EIR = RB_ETH_EIR_TXIF;
  R16_ETH_ETXLN = uip_len;
  R16_ETH_ETXST = (UINT16)ethtxbuffer;
  R8_ETH_ECON1 |= RB_ETH_ECON1_TXRTS;                                     //开始发送
  eth_irq_flag = R8_ETH_EIR;
  while(!(eth_irq_flag&RB_ETH_EIR_TXIF))                                        //发送完成中断
  {
    //printf("eth send wait\r\n");
    eth_irq_flag = R8_ETH_EIR;
  }
  //EthernetPacketPut(ETH_BASE, uip_buf, uip_len);
}



/********************************** (C) COPYRIGHT *******************************
* File Name          : CH579XETH.H
* Author             : WCH #define
* Version            : V1.0
* Date               : 2017/12/15
* Description        : CH579网络相关寄存器定义和函数定义，之后将其分开，注释长度
   统一按照80个字符分割
*******************************************************************************/
#ifndef __ETHSFR_H__
#define __ETHSFR_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/                                             
#define ETH_BASE_ADDR           (0x40009000)

//中断设置
#define R8_ETH_EIE              (*((PUINT8V)(ETH_BASE_ADDR+0x03)))              /* 中断使能寄存器 */
#define  RB_ETH_EIE_INTIE       0x80                                            /* RW 中断使能 */
#define  RB_ETH_EIE_RXIE        0x40                                            /* RW 接收完成中断使能 */
#define  RB_ETH_EIE_LINKIE      0x10                                            /* RW Link 变化中断使能 */
#define  RB_ETH_EIE_TXIE        0x08                                            /* RW 发送完成中断使能 */
#define  RB_ETH_EIE_R_EN50      0x04                                            /* RW TX 50Ω电阻调节。1：片内 50Ω连接 0：片内 50Ω断开 */
#define  RB_ETH_EIE_TXERIE      0x02                                            /* RW 发送错误中断使能 */
#define  RB_ETH_EIE_RXERIE      0x01                                            /* RW 接收错误中断使能 */
//状态
//#define R32_ETH_CON             (*((PUINT32V)(ETH_BASE_ADDR+0x04)))
#define R8_ETH_EIR              (*((PUINT8V)(ETH_BASE_ADDR+0x04)))              /* 中断标志寄存器 */   
#define  RB_ETH_EIR_RXIF        0x40                                            /* RW1 接收完成标志 */
#define  RB_ETH_EIR_LINKIF      0x10                                            /* RW1 Link 变化标志 */
#define  RB_ETH_EIR_TXIF        0x08                                            /* RW1 发送完成标志 */
#define  RB_ETH_EIR_TXERIF      0x02                                            /* RW1 发送错误标志 */
#define  RB_ETH_EIR_RXERIF      0x01                                            /* RW1 接收错误标志 */
#define R8_ETH_ESTAT            (*((PUINT8V)(ETH_BASE_ADDR+0x05)))              /* 状态寄存器 */
#define  RB_ETH_ESTAT_INT       0x80                                            /* RW1 中断 */
#define  RB_ETH_ESTAT_BUFER     0x40                                            /* RW1 Buffer 错误，理论上 mcu 主频太低才会发生 */
#define  RB_ETH_ESTAT_RXCRCER   0x20                                            /* RO 接收 crc 出错 */
#define  RB_ETH_ESTAT_RXNIBBLE  0x10                                            /* RO 接收 nibble 错误 */
#define  RB_ETH_ESTAT_RXMORE    0x08                                            /* RO 接收超过最大数据包 */
#define  RB_ETH_ESTAT_RXBUSY    0x04                                            /* RO 接收进行中 */
#define  RB_ETH_ESTAT_TXABRT    0x02                                            /* RO 发送被 mcu 打断 */
#define R8_ETH_ECON2            (*((PUINT8V)(ETH_BASE_ADDR+0x06)))              /* PHY 模拟模块控制寄存器 */
#define  RB_ETH_ECON2_RX        0x0E                                /* Ҙѫдɫ011 */
#define  RB_ETH_ECON2_TX        0x01
#define  RB_ETH_ECON2_MUST      0x06                                /* Ҙѫдɫ011 */
#define  RB_ETH_ECON2_RX_VCM    0xe0
//#define RX_VCM2               0x80                                            /* RW 测试模式 */
//#define RX_VCM1               0x40
//#define RX_VCM0               0x20
#define  RB_ETH_ECON2_RX_REF    0x1c
//#define RX_REF2               0x10                                            /* RW RX 参考电压值 000: 25mV 001: 49mV 010: 74mV*/
//#define RX_REF1               0x08                                            /* 011: 98mV 100: 123mV 101: 148mV */
//#define RX_REF0               0x04                                            /* 110: 173mV 111: 198mV */
#define  RB_ETH_ECON2_TX_AMP    0x03
//#define TX_AMP1               0x02                                            /* RW TX DRIVER 偏置电流 00: 43mA / 14.5mA 01: 53.1mA /18mA */
//#define TX_AMP0               0x01                                            /* 10: 75.6 mA/25.6mA 11: 122 mA /41.45mA */
#define R8_ETH_ECON1            (*((PUINT8V)(ETH_BASE_ADDR+0x07)))              /* 收发控制寄存器 */
#define  RB_ETH_ECON1_TXRST     0x80                                            /* RW 发送模块复位 */
#define  RB_ETH_ECON1_RXRST     0x40                                            /* RW 接收模块复位 */
#define  RB_ETH_ECON1_TXRTS     0x08                                            /* RW 发送开始，发送完成后自动清零，如主动清零会使发送错误标志TXERIF和TXABRT变1 */
#define  RB_ETH_ECON1_RXEN      0x04                                            /* RW 接收使能，清零时如正在接受则错误标志RXERIF变1 */
//发送控制
//#define R32_ETH_TX              (*((PUINT32V)(ETH_BASE_ADDR+0x08)))             /* 发送控制 */
#define R16_ETH_ETXST           (*((PUINT16V)(ETH_BASE_ADDR+0x08)))             /* RW 发送 DMA 缓冲区起始地址 */
#define R8_ETH_ETXSTL           (*((PUINT8V)(ETH_BASE_ADDR+0x08)))              /* RW 发送 DMA 缓冲区起始地址低字节 */
#define R8_ETH_ETXSTH           (*((PUINT8V)(ETH_BASE_ADDR+0x09)))              /* RW 发送 DMA 缓冲区起始地址高字节，低2位固定为0 */
#define R16_ETH_ETXLN           (*((PUINT16V)(ETH_BASE_ADDR+0x0a)))             /* RW 发送长度 */
#define R8_ETH_ETXLNL           (*((PUINT8V)(ETH_BASE_ADDR+0x0a)))              /* RW 发送长度低字节 */
#define R8_ETH_ETXLNH           (*((PUINT8V)(ETH_BASE_ADDR+0x0b)))              /* RW 发送长度高字节 */
//接收控制
//#define R32_ETH_RX              (*((PUINT32V)(ETH_BASE_ADDR+0x0c)))
#define R16_ETH_ERXST           (*((PUINT16V)(ETH_BASE_ADDR+0x0c)))             /* RW 接收 DMA 缓冲区起始地址 */
#define R8_ETH_ERXSTL           (*((PUINT8V)(ETH_BASE_ADDR+0x0c)))              /* RW 接收 DMA 缓冲区起始地址低字节 */
#define R8_ETH_ERXSTH           (*((PUINT8V)(ETH_BASE_ADDR+0x0d)))              /* RW 接收 DMA 缓冲区起始地址高字节，低2位固定为0 */
#define R16_ETH_ERXLN           (*((PUINT16V)(ETH_BASE_ADDR+0x0e)))             /* RO 接收长度 */
#define R8_ETH_ERXLNL           (*((PUINT8V)(ETH_BASE_ADDR+0x0e)))              /* RO 接收长度低字节 */
#define R8_ETH_ERXLNH           (*((PUINT8V)(ETH_BASE_ADDR+0x0f)))              /* RO 接收长度高字节 */
//hash表
#define R32_ETH_HTL             (*((PUINT32V)(ETH_BASE_ADDR+0x10)))
#define R8_ETH_EHT0             (*((PUINT8V)(ETH_BASE_ADDR+0x10)))              /* RW Hash Table Byte0 */
#define R8_ETH_EHT1             (*((PUINT8V)(ETH_BASE_ADDR+0x11)))              /* RW Hash Table Byte1 */
#define R8_ETH_EHT2             (*((PUINT8V)(ETH_BASE_ADDR+0x12)))              /* RW Hash Table Byte2 */
#define R8_ETH_EHT3             (*((PUINT8V)(ETH_BASE_ADDR+0x13)))              /* RW Hash Table Byte3 */
#define R32_ETH_HTH             (*((PUINT32V)(ETH_BASE_ADDR+0x14)))
#define R8_ETH_EHT4             (*((PUINT8V)(ETH_BASE_ADDR+0x14)))              /* RW Hash Table Byte4 */
#define R8_ETH_EHT5             (*((PUINT8V)(ETH_BASE_ADDR+0x15)))              /* RW Hash Table Byte5 */
#define R8_ETH_EHT6             (*((PUINT8V)(ETH_BASE_ADDR+0x16)))              /* RW Hash Table Byte6 */
#define R8_ETH_EHT7             (*((PUINT8V)(ETH_BASE_ADDR+0x17)))              /* RW Hash Table Byte7 */
//流控制
//#define R32_ETH_MACON           (*((PUINT32V)(ETH_BASE_ADDR+0x18)))
#define R8_ETH_ERXFCON          (*((PUINT8V)(ETH_BASE_ADDR+0x18)))              /* 接收包过滤控制寄存器 */
#define  RB_ETH_ERXFCON_UCEN    0x80                                            /* RW 0=不启用该过滤条件，1=当ANDOR=1，目标地址不匹配将被过滤，当ANDOR=0，目标地址匹配将被接收 */
#define  RB_ETH_ERXFCON_ANDOR   0x40                                            /* RW 1=AND，所有过滤条件都满足包才被接收 0=OR，任一过滤条件满足包均被接收 */
#define  RB_ETH_ERXFCON_CRCEN   0x20                                            /* RW 0=不启用该过滤条件，1=当ANDOR=1，CRC校验错将被过滤，当ANDOR=0，CRC校验正确将被接收 */
#define  RB_ETH_ERXFCON_MPEN    0x08                                            /* RW 0=不启用该过滤条件，1=当ANDOR=1，非魔法包将被过滤，当ANDOR=0，魔法包将被接收 */
#define  RB_ETH_ERXFCON_HTEN    0x04                                            /* RW 0=不启用该过滤条件，1=当ANDOR=1，hash table不匹配将被过滤，当ANDOR=0，hash table匹配将被接收 */
#define  RB_ETH_ERXFCON_MCEN    0x02                                            /* RW 0=不启用该过滤条件，1=当ANDOR=1，组播包不匹配将被过滤，当ANDOR=0，组播包匹配将被接收 */
#define  RB_ETH_ERXFCON_BCEN    0x01                                            /* RW 0=不启用该过滤条件，1=当ANDOR=1，非广播包将被过滤，当ANDOR=0，广播包将被接收 */
#define R8_ETH_MACON1           (*((PUINT8V)(ETH_BASE_ADDR+0x19)))              /* Mac 层流控制寄存器 */
#define  RB_ETH_MACON1_FCEN     0x30                                            /* RW 当FULDPX=0均无效，当FULDPX=1，11=发送0 timer暂停帧，然后停止发送，10=周期性发送暂停帧，01=发送一次暂停帧，然后停止发送，00=停止发送暂停帧 */
#define  RB_ETH_MACON1_TXPAUS   0x08                                            /* RW 发送pause帧使能 */
#define  RB_ETH_MACON1_RXPAUS   0x04                                            /* RW 接收pause帧使能 */
#define  RB_ETH_MACON1_PASSALL  0x02                                            /* RW 1=没被过滤的控制帧将写入缓存，0=控制帧将被过滤 */
#define  RB_ETH_MACON1_MARXEN   0x01                                            /* RW MAC层接收使能 */
#define R8_ETH_MACON2           (*((PUINT8V)(ETH_BASE_ADDR+0x1a)))              /* Mac 层封包控制寄存器 */
#define  RB_ETH_MACON2_PADCFG   0xe0                                            /* RW 短包填充设置 */
#define  RB_ETH_MACON2_TXCRCEN  0x10                                            /* RW 发送添加crc，PADCFG中如需要添加crc，该位置1 */
#define  RB_ETH_MACON2_PHDREN   0x08                                            /* RW 特殊4字节不参与crc校验 */
#define  RB_ETH_MACON2_HFRMEN   0x04                                            /* RW 允许接收巨型帧 */
#define  RB_ETH_MACON2_FULDPX   0x01                                            /* RW 全双工 */
#define R8_ETH_MABBIPG          (*((PUINT8V)(ETH_BASE_ADDR+0x1b)))              /* 最小包间间隔寄存器 */
#define  RB_ETH_MABBIPG_MABBIPG 0x7f                                            /* RW 最小包间间隔字节数 */
//#define R32_ETH_TIM             (*((PUINT32V)(ETH_BASE_ADDR+0x1c)))
#define R16_ETH_EPAUS           (*((PUINT16V)(ETH_BASE_ADDR+0x1c)))             /* RW 流控制暂停帧时间寄存器 */
#define R8_ETH_EPAUSL           (*((PUINT8V)(ETH_BASE_ADDR+0x1c)))              /* RW 流控制暂停帧时间寄存器低字节 */
#define R8_ETH_EPAUSH           (*((PUINT8V)(ETH_BASE_ADDR+0x1d)))              /* RW 流控制暂停帧时间寄存器高字节 */
#define R16_ETH_MAMXFL          (*((PUINT16V)(ETH_BASE_ADDR+0x1e)))             /* RW 最大接收包长度寄存器 */
#define R8_ETH_MAMXFLL          (*((PUINT8V)(ETH_BASE_ADDR+0x1e)))              /* RW 最大接收包长度寄存器低字节 */
#define R8_ETH_MAMFLH           (*((PUINT8V)(ETH_BASE_ADDR+0x1f)))              /* RW 最大接收包长度寄存器高字节 */
//MII控制
#define R16_ETH_MIRD            (*((PUINT16V)(ETH_BASE_ADDR+0x20)))             /* RW MII 读寄存器 */
#define R8_ETH_MIRDL            (*((PUINT8V)(ETH_BASE_ADDR+0x20)))              /* RW MII 读寄存器低字节 */
#define R8_ETH_MIRDH            (*((PUINT8V)(ETH_BASE_ADDR+0x21)))              /* RW MII 读寄存器高字节 */
//#define R32_ETH_MIWR            (*((PUINT32V)(ETH_BASE_ADDR+0x24)))             /* WO MII 写寄存器 */
#define R8_ETH_MIREGADR         (*((PUINT8V)(ETH_BASE_ADDR+0x24)))              /* MII 寄存器地址 */
#define  RB_ETH_MIREGADR_MIIWR  0x20                                            /* WO MII 写命令 */
#define  RB_ETH_MIREGADR_MIRDL  0x1f                                            /* RW PHY 寄存器地址 */    
/* 状态呢？？*/
#define R8_ETH_MISTAT           (*((PUINT8V)(ETH_BASE_ADDR+0x25)))              /* MII 状态寄存器 */

#define R16_ETH_MIWR            (*((PUINT16V)(ETH_BASE_ADDR+0x26)))             /* WO MII 写寄存器 */
#define R8_ETH_MIWRL            (*((PUINT8V)(ETH_BASE_ADDR+0x26)))              /* WO MII 写寄存器低字节 */
#define R8_ETH_MIWRH            (*((PUINT8V)(ETH_BASE_ADDR+0x27)))              /* WO MII 写寄存器高字节 */
//MAC地址
//#define R32_ETH_MAADRL          (*((PUINT32V)(ETH_BASE_ADDR+0x28)))             /* RW MAC 1-4 */
#define R8_ETH_MAADR1           (*((PUINT8V)(ETH_BASE_ADDR+0x28)))              /* RW MAC Address Byte1 */
#define R8_ETH_MAADR2           (*((PUINT8V)(ETH_BASE_ADDR+0x29)))              /* RW MAC Address Byte2 */
#define R8_ETH_MAADR3           (*((PUINT8V)(ETH_BASE_ADDR+0x2a)))              /* RW MAC Address Byte3 */
#define R8_ETH_MAADR4           (*((PUINT8V)(ETH_BASE_ADDR+0x2b)))              /* RW MAC Address Byte4 */
//#define R16_ETH_MAADRH          (*((PUINT16V)(ETH_BASE_ADDR+0x2C)))             /* RW MAC 5-6 */
#define R8_ETH_MAADR5           (*((PUINT8V)(ETH_BASE_ADDR+0x2c)))              /* RW MAC Address Byte5 */
#define R8_ETH_MAADR6           (*((PUINT8V)(ETH_BASE_ADDR+0x2d)))              /* RW MAC Address Byte6 */
//PHY地址 
#define PHY_BMCR                0x00                                            /* Control Register */
#define PHY_BMSR                0x01                                            /* Status Register */
#define PHY_ANAR                0x04                                            /* Auto-Negotiation Advertisement Register */
#define PHY_ANLPAR              0x05                                            /* Auto-Negotiation Link Partner Base  Page Ability Register*/
#define PHY_ANER                0x06                                            /* Auto-Negotiation Expansion Register */
#define PHY_MDIX                0x1e                                            /* 自定义 MDIX 模式寄存器 */
//自定义MDIX模式寄存器  @PHY_MDIX
#define PN_NORMAL               0x04                                            /* 模拟p，n极性选择 */
#define MDIX_MODE_MASK          0x03                                            /* mdix设置 */
#define MDIX_MODE_AUTO          0x00                                            /*  */
#define MDIX_MODE_MDIX          0x01
#define MDIX_MODE_MDI           0x02
//ECON2测试模式，待定  
#define RX_VCM_MODE_0
#define RX_VCM_MODE_1
#define RX_VCM_MODE_2
#define RX_VCM_MODE_3
//RX 参考电压值 设置  @RX_REF
#define RX_REF_25mV             (0<<2)                                          /* 25mV */
#define RX_REF_49mV             (1<<2)                                          /* 49mV */
#define RX_REF_74mV             (2<<2)                                          /* 74mV */
#define RX_REF_98mV             (3<<2)                                          /* 98mV */
#define RX_REF_123mV            (4<<2)                                          /* 123mV */
#define RX_REF_148mV            (5<<2)                                          /* 148mV */
#define RX_REF_173mV            (6<<2)                                          /* 173mV */
#define RX_REF_198mV            (7<<2)                                          /* 198mV */
//TX DRIVER 偏置电流  @TX_AMP
#define TX_AMP_0                (0<<0)                                          /* 43mA   / 14.5mA   (1.4V/0.7V) */
#define TX_AMP_1                (1<<0)                                          /* 53.1mA / 18mA     (1.8V/0.9V) */
#define TX_AMP_2                (2<<0)                                          /* 75.6mA / 25.6mA   (2.6V/1.3V) */
#define TX_AMP_3                (3<<0)                                          /* 122mA  / 41.45mA  (4.1V/2.3V) */
//FCEN暂停帧控制      @FCEN
#define FCEN_0_TIMER            (3<<4)                                          /* 发送 0 timer 暂停帧，然后停止发送 */
#define FCEN_CYCLE              (2<<4)                                          /* 周期性发送暂停帧 */
#define FCEN_ONCE               (1<<4)                                          /* 发送一次暂停帧，然后停止发送 */
#define FCEN_STOP               (0<<4)                                          /* 停止发送暂停帧 */
//PADCFG短包控制  @PADCFG
#define PADCFG_AUTO_0           (7<<5)                                          /* 所有短包填充00h至64字节，再4字节crc */
#define PADCFG_NO_ACT_0         (6<<5)                                          /* 不填充短包 */
#define PADCFG_DETE_AUTO        (5<<5)                                          /* 检测到字段为8100h的VLAN网络包自动填充00h至64字节，否则短包填充60字节0，填充后再4字节crc */
#define PADCFG_NO_ACT_1         (4<<5)                                          /* 不填充短包 */
#define PADCFG_AUTO_1           (3<<5)                                          /* 同111 */
#define PADCFG_NO_ACT_2         (2<<5)                                          /* 不填充短包 */
#define PADCFG_AUTO_3           (1<<5)                                          /* 所有短包填充00h至60字节，再4字节crc */
#define PADCFG_NO_ACT_3         (0<<5)                                          /* 不填充短包 */

/*This define allow to customize configuration of the Ethernet driver buffers */
#define CUSTOM_DRIVER_BUFFERS_CONFIG

/** @defgroup ENET_Buffers_setting 
  * @{
  */ 
#define TCP_MSS         (1500 - 40)
#define ETH_MAX_PACKET_SIZE    (ETH_HEADER + ETH_EXTRA + VLAN_TAG + MAX_ETH_PAYLOAD + ETH_CRC)//1524    /*!< ETH_HEADER + ETH_EXTRA + VLAN_TAG + MAX_ETH_PAYLOAD + ETH_CRC */
#define ETH_HEADER               14    /*!< 6 byte Dest addr, 6 byte Src addr, 2 byte length/type */
#define ETH_CRC                   4    /*!< Ethernet CRC */
#define ETH_EXTRA                 2    /*!< Extra bytes in some cases */   
#define VLAN_TAG                  4    /*!< optional 802.1q VLAN Tag */
#define MIN_ETH_PAYLOAD          46    /*!< Minimum Ethernet payload size */
#define MAX_ETH_PAYLOAD        (TCP_MSS + 40)    /*!< Maximum Ethernet payload size */
#define JUMBO_FRAME_PAYLOAD    9000    /*!< Jumbo frame payload size */      

#ifdef  CUSTOM_DRIVER_BUFFERS_CONFIG
/* Redefinition of the Ethernet driver buffers size and count */
 #define ETH_RX_BUF_SIZE    ETH_MAX_PACKET_SIZE  /* buffer size for receive */
 #define ETH_TX_BUF_SIZE    ETH_MAX_PACKET_SIZE  /* buffer size for transmit */
 //#define ETH_RXBUFNB        4                    /* 4 Rx buffers of size ETH_RX_BUF_SIZE */
 //#define ETH_TXBUFNB        4                    /* 4 Tx buffers of size ETH_TX_BUF_SIZE */
 #define ETH_BUFNB        3//5
#endif

//网络收发MAC层配置 1536
#define ETHER_HEAD_LEN          14                                              /* 网络帧头长度 */
#define MACADDR_LEN             6
#define MAC_MAX_LEN             ETH_MAX_PACKET_SIZE                                            /* MAC层最大包长度 */
#define RX_MAX_BUF_LEN          ETH_RX_BUF_SIZE                                            /* 最大接收包数据长度 */
#define RX_BUF_SIZE             ETH_RX_BUF_SIZE                                            /* 接收缓存大小 */
#define TX_BUF_SIZE             ETH_TX_BUF_SIZE                                            /* 发送缓存大小 */
#define RX_QUEUE_NUM            2                                               /* 接收缓冲区队列数 */
#define TX_QUEUE_NUM            2                                               /* 发送缓冲区队列数 */

//MAC数据包格式定义
typedef __packed struct _MAC_FRAM
{
    UINT8   DestMAC[MACADDR_LEN];
    UINT8   SourMAC[MACADDR_LEN];
    UINT16  PROTOCOL;
#define ETHER_TYPE_IP           0x0800    
#define ETHER_TYPE_ARP          0x0806
    UINT8   Data[MAC_MAX_LEN];      //数据
}MAC_FRAM;

#define IPDataLen      1484   //修改IP的数据包包长
#define TCPDataLen     1464   //修改TCP的数据包包长
#define UDPDataLen     1484   //修改UDP的数据包包长

typedef __packed  struct _ARP_FRAM
{
    UINT16 HardwareType;
    UINT16 ProtlType;
    UINT8  HardwareSize;
    UINT8  ProtlSize;
    UINT16 OpCode;             //操作码  0x0001:请求  0x0002应答
    UINT8  SenderMAC[6];       //发送者的MAC
    UINT8  SenderIP[4];
    UINT8  TargetMAC[6];       //接收者的信息
    UINT8  TargetIP[4];
}ARP_FRAM;

typedef __packed  struct _IP_FRAM
{
    UINT8  VerAndLen;        //4bit版本 4bit首部长度
    UINT8  TOS;              //8bit服务类型
    UINT16 TotalLen;         //16位总长度-->
    UINT16 Flag;             //16位标识
    UINT16 FlagAndShift;     //3bit标志 13bit偏移 bit15:保留 bit14：分片标志（0：分片；1：没有分片） bit：13：最后一包数据(0:最后一包；1：其他的包)，偏移的字节数是 = 偏移值*8
    UINT8  TTL;
    UINT8  Protocol;         //协议
    UINT16 CheckSum;         //校验和
    UINT8  SourIP[4];        //源IP
    UINT8  DesIP[4];         //目的IP
    UINT8  Data[IPDataLen];  //数据
}IP_FRAM;


typedef __packed struct _TCP_FRAM
{
    UINT16 SrcPort;          //源端口号
    UINT16 DstPort;          //目的端口号
    UINT32 SequeNum;         //序列号
    UINT32 AckNum;           //确认号  -->过0如何记录？
    UINT8  DataOffset;       //长度-->
    UINT8  Flags;            //标志
    UINT16 Window;           //窗口大小
    UINT16 CheckSum;         //
    UINT16 UrgenPointer;     //
    UINT8  Data[TCPDataLen]; //数据
}TCP_FRAM;

typedef __packed struct _UDP_FRAM
{
    UINT16 SrcPort;          //源端口号
    UINT16 DstPort;          //目的端口号
    UINT16 DataLength;
    UINT16 CheckSum;         //
//  UINT16 UrgenPointer;     //
    UINT8  Data[UDPDataLen]; //数据
}UDP_FRAM;



//收发管理定义
typedef __packed struct _RXBUFST
{
    UINT8  RecvEn;                                                              /* 使能控制 */
    UINT8  RecvIndex;                                                           /* MAC层接收缓冲区的索引 */
    UINT8  ReadIndex;                                                           /* 读取缓冲区的索引值 */
    UINT8  RemainCout;                                                          /* 剩余未读取的数据包 */
    UINT8  RxBufStau[RX_QUEUE_NUM];                                             /* 接收缓冲区的状态 */
    UINT16 RxBufLen[RX_QUEUE_NUM];                                              /* 接收缓冲区数据的有效长度 */
    UINT32 RxBufAddr[RX_QUEUE_NUM];                                             /* 接收缓冲区的起始地址 */
}RXBUFST;

typedef __packed struct _TXBUFST
{
    UINT8  SendEn;                                                              /* 使能控制 */
    UINT8  SendIndex;                                                           /* MAC层发送缓冲区的索引 */
    UINT8  WriteIndex;                                                          /* 发送缓冲区空闲的索引值 */
    UINT8  TxQueueCnt;                                                          /* 发送队列中排队的数量 */
    UINT8  TxBufStau[TX_QUEUE_NUM];                                             /* 发送缓冲区的状态 */
    UINT16 TxBufLen[TX_QUEUE_NUM];                                              /* 发送缓冲区数据的有效长度 */
    UINT32 TxBufAddr[TX_QUEUE_NUM];                                             /* 发送缓冲区的起始地址 */
}TXBUFST;

#define ETH_BUF_STATE_IDLE                      (1 << 0)
#define ETH_BUF_STATE_DMA_OWN                   (1 << 1)
#define ETH_BUF_STATE_LWIP_OWN                  (1 << 2)

#ifndef RESET
#define RESET   0
#endif

#ifndef SET
#define SET     1
#endif

/* The definition of the frame tx & rx list. */
struct framelistitem
{
    struct framelistitem *next;
    UINT8 buffer[ETH_MAX_PACKET_SIZE];
    UINT32 len;
    //UINT32 status;
};
typedef struct framelistitem st_framelistitem;

extern st_framelistitem * current_dma_tx_fram_point;
extern st_framelistitem * current_dma_rx_fram_point;

//对外接口函数
void ETHInit(void);
void ETHParaInit(void);
void ETH_IRQ_Deal(void);
void ETHSetRx(uint8_t enable);

UINT8 ETHSendChk(void);
//void ETHSend(struct pbuf*);
UINT8 ETHRecCnt(void);

#ifdef __cplusplus
}
#endif

#endif

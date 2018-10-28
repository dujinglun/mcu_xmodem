
/******************************************************************************************  
软件驱动程序(C8051F020)  
SST25VF016B 16 Mbit(2M x 8) 串行Flash存储器  
(2007年4月19日)  
关于此程序:  
此程序为用户使用该FLASH提供了一个完整功能函数库,用户可根据自己的需要使用这些函数   
------------------------------------------------------------------  
init                    初始化时钟,进入模式0  
Send_Byte               用MOSI发送一字节(上升沿)  
Get_Byte                用MISO接收一字节(下降沿)  
Poll_SO                 使用MISO作为AAI模式下的RY/BY#输出  
芯片的功能函数包:  
------------------------------------------------------------------  
Read_Status_Register    读取状态寄存器  
EWSR                    使能写状态寄存器  
WRSR                    改写状态寄存器  
WREN                    写使能  
WRDI                    写禁止  
EBSY                    允许使用MISO 作为AAI模式下RY/BY#的输出脚  
DBSY                    禁止用MISO输出RY/BY#  
Read_ID                 读取厂商ID或芯片ID  
Jedec_ID_Read           读取全部ID  
Read                    读取一个字节,返回byte(最大25 MHz CLK的频率)  
Read_Cont               读取连续地址内的数据(最大25 MHz时钟频率)  
HighSpeed_Read          读取一个字节(最大50Mhz时钟频率)并返回byte  
HighSpeed_Read_Cont     连续读取(最大50 MHz时钟频率)  
Byte_Program            写一个字节数据  
Auto_Add_IncA           初始化Auto Address Increment(AAI)  
Auto_Add_IncB           AAI初始化后进入Auto_Address_Increment(AAI)  
Auto_Add_IncA_EBSY      带EBSY的初始化Auto Address Increment(AAI)   
Auto_Add_IncB_EBSY      带EBSY的Auto_Address_Increment (AAI)  
Chip_Erase              擦除整个芯片  
Sector_Erase            擦除一个扇区(4 KB)  
Block_Erase_32K         擦除一块32 KByte的区域  
Block_Erase_64K         擦除一块64 KByte的区域  
Wait_Busy               等待空闲(状态寄存器的BUSY位为0)  
Wait_Busy_AAI           AAI模式下等待空闲  
WREN_Check              检查WEL是否被置位  
WREN_AAI_Check          检查WEL和AAI模式位被置位  
   
************************************************************************/   
/********************************************************************/   
/* Copyright Silicon Storage Technology, Inc. (SST), 1994-2005      */   
/* Example "C" language Driver of SST25VF016B Serial Flash      */   
/* Conrado Canio, Silicon Storage Technology, Inc.                  */   
/*                                                                  */   
/* Revision 1.0, August 1st, 2005                   */      
/*                                                                  */   
/*                                  */   
/********************************************************************/   
//#include <stdlib.h>   
#include <C8051F020.h>   
#include "SST25VF016B_SOFT.H"   
   
#define uchar unsigned char   
#define uint unsigned int   
#define ulong unsigned long   
   
uchar xdata upper_128[128]; /* 存取读到的数据,发在外部RAM中 */   
/************************************************************************/   
/* 程序名称:    init                            */   
/*                                  */   
/* 程序功能:    用来将时钟线初始化为低. 必须在将设备设置为模式0之前调用*/   
/* 输入      :    无                           */   
/*                                  */   
/* 输出      :    SCK                         */   
/************************************************************************/   
void init()   
{   
    uchar i;   
    P7 |= 0x7f; /* 设置SCK为低电平初始化状态 */   
    for(i=255;i>0;i--);   
    P7 &= 0x80;   
}   
/***********************************************************************/   
/*程序名称: SPI_CFG****/   
/*功能:       用来配置SPI */   
/************************************************************************/   
void SPI_CFG(uchar spicfg,uchar spickr,uchar spicn)   
{   
    SPI0CFG = spicfg;   
    SPI0CKR = spickr;   
    SPI0CN  = spicn;   
    EIE1 |= 0X01;   
}   
   
   
/************************************************************************/   
/* 程序名称: Send_Byte                          */   
/* 程序功能:在时钟的上升沿在MOSI线上写入一位数据,写满一个字节         */   
/* 输入:out                           */   
/* 输出:SI                            */   
/************************************************************************/   
void Send_Byte(uchar out)   
{   
       
    uchar i;   
    for(i=0;i<3;i++);   
    SPI0DAT =out;   
    while(TXBSY);   
   
}   
/************************************************************************/   
/* 程序名称: Get_Byte                           */   
/* 程序功能：在SCL的下降沿从MISO线上读取数据     */   
/* 输入:  SO                          */   
/* 输出:  None                            */   
/************************************************************************/   
uchar Get_Byte()   
 {    
    uchar in;   
    Send_Byte(0x00);//产生8位移位脉冲   
    in = SPI0DAT;   
    return in;   
}   
   
/************************************************************************/   
/* 程序名称: Poll_SO                            */   
/* 程序功能: 在AAI模式下监测MISO线是否变为1,用以显示AAI操作模式完成**/   
/*  输入: SO                          */   
/*  输出: None                            */   
/************************************************************************/   
void Poll_SO()   
{   
    uchar temp = 0;   
    CE =0;   
    while (temp == 0x00)    /* waste time until not busy */   
    temp = SO;   
    CE =1;   
}   
/************************************************************************/   
/************************************************************************/   
/* 程序名称: Read_Status_Register                   */   
/* 程序功能: 用来读取状态寄存器,并返回状态寄存器的值   */   
/* 输入:      None                            */   
/* 返回:      byte                            */   
/************************************************************************/   
uchar Read_Status_Register()   
{   
    uchar byte = 0;   
    CE =0;          /* 使能设备 */   
    Send_Byte(0x05);        /* 发送读状态寄存器的命令 */   
    byte = Get_Byte();      /* 读取状态寄存器 */   
    CE =1;          /* 禁止设备 */   
    return byte;   
}   
/************************************************************************/   
/* 程序名称: EWSR                           */   
/* 程序功能: 使能改写状态寄存器操作            */   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void EWSR()   
{   
    CE =0;          /* 使能设备*/   
    Send_Byte(0x50);        /* 发送使能写寄存器的命令 */   
    CE =1;          /* 禁止设备*/   
}   
/************************************************************************/   
/* 程序名称: WRSR                           */   
/* 程序功能: 往状态寄存器里写一个字节           *  
/* 输入:      byte                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void WRSR(byte)   
{   
    CE =0;          /* 使能设备 */   
    Send_Byte(0x01);        /* 发送写状态寄存器 */   
    Send_Byte(byte);        /* 改变寄存器里BPx或者BPL (只有2,3,4,5,7位可以改写) */   
    CE =1;          /* 禁止设备 */   
}   
/************************************************************************/   
/* 程序名称: WREN                           */   
/* 程序功能: 写使能  同样可以用于使能写状态寄存器    */   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void WREN()   
{   
    CE =0;             
    Send_Byte(0x06);        /* 发送WREN命令 */   
    CE =1;             
}   
/************************************************************************/   
/* 名称: WRDI                         */   
/* 功能: 写禁止*/   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void WRDI()   
{   
    CE =0;             
    Send_Byte(0x04);        /* 发送WRDI命令*/   
    CE =1;         
}   
/************************************************************************/   
/* 名称: EBSY                         */   
/* 功能: 允许MISO在AAI模式期间输出RY/BY# 状态 */   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void EBSY()   
{   
    CE =0;             
    Send_Byte(0x70);        /* 发送EBSY命令*/   
    CE =1;         
}   
/************************************************************************/   
/* 名称: DBSY                         */   
/* 功能: 禁止MISO在AAI模式下作为输出RY/BY#状态的信号*/   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void DBSY()   
{   
    CE =0;         
    Send_Byte(0x80);        /* 发送DBSY命令 */   
    CE =1;         
}   
/************************************************************************/   
/* 名称: Read_ID                          */   
/* 功能:用于读取厂商ID 和设备ID.使用90h 或ABh 命令读取ID.   */   
/*      由用户提供ID_addr ,用以决定是否先输出厂商ID或设备ID        */   
/* 输入:  ID_addr                         */   
/* 返回:  byte:   ID1(厂商ID = BFh 或设备 ID = 80h)    */   
/************************************************************************/   
uchar Read_ID(ID_addr)   
{   
    uchar byte;   
    CE =0;                     
    Send_Byte(0x90);        /* 发送read ID 指令 (90h or ABh) */   
    Send_Byte(0x00);        /* 发送地址 */   
    Send_Byte(0x00);        /* 发送地址 */   
    Send_Byte(ID_addr);     /* 发送地址00H 或01H */   
    byte = Get_Byte();      /* 接收 */   
    CE =1;                 
    return byte;   
}   
/************************************************************************/   
/* 名称: Jedec_ID_Read                        */   
/* 功能: 读取厂商ID(BFh),记忆体类型(25h),设备ID(41h). 使用9Fh作为JEDEC ID命令.     */   
/* 输入:      None                            */   
/* 返回:      IDs_Read:ID1(厂商ID = BFh, 记忆体ID(25h), 设备ID(80h)*/   
/************************************************************************/   
ulong Jedec_ID_Read()    
{   
    ulong temp;   
    temp = 0;   
    CE =0;           /* enable device */   
    Send_Byte(0x9F);         /* send JEDEC ID command (9Fh) */   
    temp = (temp | Get_Byte()) << 8; /* receive byte */   
    temp = (temp | Get_Byte()) << 8;     
    temp = (temp | Get_Byte());      /* temp value = 0xBF2541 */   
    CE =1;           /* disable device */   
    return temp;   
}   
/************************************************************************/   
/* 名称: Read                         */     
/* 功能: 读取一个地址内一个字节的数据.返回读取的数据*/   
/* 输入:      Dst:    Destination Address 000000H - 1FFFFFH       */   
/* 返回:      byte                            */   
/************************************************************************/   
uchar Read(ulong Dst)    
{   
    uchar byte = 0;    
    CE =0;          /* enable device */   
    Send_Byte(0x03);        /* read command */   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* send 3 address bytes */   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    byte = Get_Byte();   
    CE =1;          /* disable device */   
    return byte;            /* return one byte read */   
}   
/************************************************************************/   
/* 名称: Read_Cont                        */         
/* 功能: 读取芯片内连续地址内的数据,最多128字节*/   
/* 输入: Dst-目标地址 (000000H - 1FFFFFH  )*/   
/************************************************************************/   
void Read_Cont(ulong Dst, ulong no_bytes)   
{   
    ulong i = 0;   
    CE =0;              /* 使能芯片 */   
    Send_Byte(0x03);            /* 发送读命令*/   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送3字节的地址 */   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    for (i = 0; i < no_bytes; i++)       /* 读取数据 */   
    {   
        upper_128[i] = Get_Byte();  /* 读取数据存放在80H - FFH */   
    }   
    CE =1;              /* 禁止芯片 */   
}   
/************************************************************************/   
/* 名称: HighSpeed_Read                       */     
/* 功能: 高速读取一个字节  */   
/* 输入: Dst(目标地址 000000H - 1FFFFFH)      */   
/* 返回: byte                         */   
/************************************************************************/   
uchar HighSpeed_Read(ulong Dst)    
{   
    uchar byte = 0;    
    CE =0;          /*  使能芯片  */   
    Send_Byte(0x0B);        /* 发送指令 */   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送3字节的地址*/   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    Send_Byte(0xFF);        /*虚拟字节*/   
    byte = Get_Byte();   
    CE =1;          /* 禁止芯片 */   
    return byte;            /* 返回读取的一个字节 */   
}   
/************************************************************************/   
/* 名称:  HighSpeed_Read_Cont                 */   
/* 功能:  高速读取芯片连续地址的内容,最大可读取128字节    */   
/* 输入:  Dst:(目标地址 000000H - 1FFFFFH)*/   
/*        no_bytes: 读取字节数   (最大128字节)   */   
/* 返回:  Nothing                         */   
/************************************************************************/   
void HighSpeed_Read_Cont(ulong Dst, ulong no_bytes)   
{   
    ulong i = 0;   
    CE =0;              /* 芯片使能 */   
    Send_Byte(0x0B);            /* 发送读取指令 */   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送3字节的地址 */   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    Send_Byte(0xFF);            /*虚拟字节*/   
    for (i = 0; i < no_bytes; i++)       /* 读取no_bytes字节*/   
    {   
        upper_128[i] = Get_Byte();  /*接收数据存入 80H - FFH */   
    }   
    CE =1;              /* 禁止芯片*/   
}   
/************************************************************************/   
/* 名称: Byte_Program                     */   
/* 功能: 写一个字节数据,被写的地址必须被擦除及未被保护      */                 
/* 输入:                              */   
/*      Dst:        (目标地址 000000H - 1FFFFFH)    */   
/*      byte:       数据          */   
/* 返回:                              */   
/*      Nothing                         */   
/************************************************************************/   
void Byte_Program(ulong Dst, uchar byte)   
{   
    CE =0;              /* 芯片使能 */   
//  WREN_Check();   
    Send_Byte(0x02);            /* 发送写操作指令 */   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送3字节地址 */   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    Send_Byte(byte);            /*发送要写的数据*/   
//  Wait_Busy();   
    CE =1;                 
}   
/************************************************************************/   
/* 名称: Auto_Add_IncA                        */   
/* 功能: 写连续地址*/   
/* 输入:                              */   
/*      Dst:      (目标地址 000000H - 1FFFFFH)      */   
/*      byte1:      第1字节数据  */   
/*      byte1:      第2字节数据   */   
/* 返回:                              */   
/*      Nothing                         */   
/************************************************************************/   
void Auto_Add_IncA(ulong Dst, uchar byte1, uchar byte2)   
{   
    CE =0;                 
    Send_Byte(0xAD);            /* 发送AAI命令*/   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送地址*/   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    Send_Byte(byte1);           /* 发送第一个字节 */      
    Send_Byte(byte2);           /* 发送第二个字节 */   
    CE =1;             
}   
/************************************************************************/   
/* 名称: 工作Auto_Add_IncB                      */   
/* 功能: 连续写入2字节,使用在Auto_Address_IncA之后                   */   
/*       进入AAI模式后仅WRDI及AAI操作指令能被执行,因为SO作为RY/BY#状态输出.*/   
/* 输入:                              */   
/*      byte1:      1st byte to be programmed       */   
/*      byte2:      2nd byte to be programmed       */   
/* 返回:                              */   
/*      Nothing                         */   
/************************************************************************/   
void Auto_Add_IncB(uchar byte1, uchar byte2)   
{   
    CE =0;              
    Send_Byte(0xAD);            /* 发送AAI命令*/   
    Send_Byte(byte1);           /* 发送第一个字节 */   
    Send_Byte(byte2);           /* 发送第二个字节*/   
    CE =1;             
}   
/************************************************************************/   
/*名称:   Auto_Add_IncA_EBSY                  */   
/*功能: 像Auto_Add_IncA一样使用EBSY 和Poll_SO  检查RY/BY. */   
/*输入:                               */   
/*      Dst:        (目标地址 000000H - 1FFFFFH)    */   
/*      byte1,   byte1;     */   
/*返回:                               */   
/*      Nothing                         */   
/************************************************************************/   
void Auto_Add_IncA_EBSY(ulong Dst, uchar byte1, uchar byte2)   
{   
    EBSY();                 /* 使用SO作为RY/BY#状态输出 */     
    CE =0;              /* 使能芯片 */   
    Send_Byte(0xAD);            /* 发送AAI命令*/   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送地址*/   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    Send_Byte(byte1);           /* 发送第一个字节 */      
    Send_Byte(byte2);           /* 发送第二个字节*/   
    CE =1;             
    Poll_SO();              /* 使用SO作为RY/BY#输出*/   
}   
/************************************************************************/   
/* 名称:  Auto_Add_IncB_EBSY                  */   
/* 功能:   
/* 输入:                              */   
/*      byte1,byte2     */   
/* 返回:                              */   
/*      Nothing                         */   
/************************************************************************/   
void Auto_Add_IncB_EBSY(uchar byte1, uchar byte2)   
{   
    CE =0;             
    Send_Byte(0xAD);            /* 发送AAI命令 */   
    Send_Byte(byte1);           /* 发送第一个字节 */   
    Send_Byte(byte2);           /* 发送第二个字节 */   
    CE =1;             
    Poll_SO();              /* 用SO传送RY/BY#   */   
    WRDI();                 /* 在DBSY前退出 */   
    DBSY();                 /* 在AAI下禁止 SO 作为RY/BY#输出*/   
}   
/************************************************************************/   
/* 名称: Chip_Erase                       */   
/* 功能: 擦除整个芯片               */   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void Chip_Erase()   
{                          
    WREN_Check();   
    CE =0;             
    Send_Byte(0x60);            /* 发送 Chip Erase命令 (60h or C7h) */   
    CE =1;   
    Wait_Busy();   
    //CE =1;               
}   
/************************************************************************/   
/* 名称: Sector_Erase                     */   
/* 功能: Sector Erases the Chip.              */   
/* 输入:      Dst:        目标地址000000H - 1FFFFFH   */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void Sector_Erase(ulong Dst)   
{   
    uchar i;   
    WREN_Check();   
    init();   
    CE =0;                 
    Send_Byte(0x20);            /* 发送Sector Erase 命令 */   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送地址 */   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    for(i=100;i>0;i--);   
    CE =1;   
    Wait_Busy();               
}   
/************************************************************************/   
/* 名称: Block_Erase_32K                      */   
/* 功能: Block Erases 32 KByte of the Chip.           */   
/* 输入:      Dst:        目标地址 000000H - 1FFFFFH  */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void Block_Erase_32K(ulong Dst)   
{   
    WREN_Check();   
    CE =0;                 
    Send_Byte(0x52);            /* 发送32 KByte Block Erase命令*/   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送3bytes地址*/   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    CE =1;   
    Wait_Busy();           
}   
/************************************************************************/   
/* 名称: Block_Erase_64K                      */   
/* 功能: Block Erases 64 KByte    */   
/* 输入:      Dst:        目标地址000000H - 1FFFFFH   */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void Block_Erase_64K(ulong Dst)   
{   
    WREN_Check();   
    CE =0;             
    Send_Byte(0xD8);            /* 发送64KByte Block Erase 命令 */   
    Send_Byte(((Dst & 0xFFFFFF) >> 16));  /* 发送3 bytes地址 */   
    Send_Byte(((Dst & 0xFFFF) >> 8));   
    Send_Byte(Dst & 0xFF);   
    CE =1;   
    Wait_Busy();           
}   
/************************************************************************/   
/* 名称: Wait_Busy                            */   
/* 功能: 等待芯片空闲(在执行Byte-Program, Sector-Erase, Block-Erase, Chip-Erase操作后)*/   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void Wait_Busy()   
{   
    while ((Read_Status_Register())&0x01 == 0x01)   /* waste time until not busy */   
        Read_Status_Register();   
}   
/************************************************************************/   
/* 名称: Wait_Busy_AAI                        */   
/* 功能:  在AAI模式下等待芯片空闲   */   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void Wait_Busy_AAI()   
{   
    while (Read_Status_Register() == 0x43)  /* 等待空闲 */   
        Read_Status_Register();   
}   
/************************************************************************/   
/* 名称: WREN_Check                       */   
/* 功能: 检查擦写操作前WEL位是否为1  */   
/* 输入:      None                            */   
/* 返回:      Nothing                         */   
/************************************************************************/   
void WREN_Check()   
{   
    uchar byte;   
    byte = Read_Status_Register();  /* 读取状态register */   
    if ((byte&0x02) != 0x02)        /* 检查WEL位置位 */   
    {   
        WREN();   
            //如果未置1进行相应处理,如对其进行写使得操作   
    }   
}   
/************************************************************************/   
/* 名称: WREN_AAI_Check                       */   
/* 功能: 在AAI模式下检查AAI和WEL位    */   
/************************************************************************/   
void WREN_AAI_Check()   
{   
    uchar byte;   
    byte = Read_Status_Register();  /*读取状态寄存器*/   
    if (byte != 0x42)       /* 核实AAI 和 WEL位被置1 */   
    {   
        while(1);          
                /*如果发生错误进行相应处理*/   
    }   
}   
/************************************************************************/   
/* 名称: Verify                           */   
/* 功能: 检查是否读取正确 */   
/* 输入:                              */   
/*      byte:       byte read               */   
/*      cor_byte:   应被读取的字节 */   
/* 返回:                              */   
/*      Nothing                         */   
/************************************************************************/     
void Verify(uchar byte, uchar cor_byte)   
{   
    if (byte != cor_byte)   
    {   
        while(1);    
            /*加入显示错误 */   
    }   
}   
void SPI_ISR() interrupt 6   
{   
    if(WCOL){WCOL =0;}  //写冲突   
    if(MODF){P5 &=0x7f;MODF =0;}  //模式错误   
    if(RXOVRN){;};   
    SPIF=0;   
} 
#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include	"SPI_Flash.h"

/**************************************************************************************************
函数名称： void	CV520_SPI_Config(void)
函数功能： LPC11UXX SP1 配置
入口参数： 无
返回参数： 无
编写日期： 2013.8.13
程序作者： 夏立新
***************************************************************************************************/
void	SPI_Flash_Config(void)
{
	LPC_IOCON->PIO1_20				&=~0x07;  	    //SCK1
	LPC_IOCON->PIO1_21				&=~0x07;  	    //MISO
	LPC_IOCON->PIO1_22				&=~0x07;  	    //MOSI

	LPC_IOCON->PIO1_20				|=0x02;  	    //SCK1
	LPC_IOCON->PIO1_21				|=0x02;  	    //MISO
	LPC_IOCON->PIO1_22				|=0x02;  	    //MOSI
									  									   	
	LPC_SYSCON->PRESETCTRL 			|=(0x01 << 2);			//禁止SSP1复位
	LPC_SYSCON->SYSAHBCLKCTRL 		|=(0X01 << 18);			//SPI1 时钟开启
	LPC_SYSCON->SSP1CLKDIV			=0x02;					//SPI1 时钟分配
	LPC_SSP1->CR0					=	(0x07 << 0)	   		//8-BIT
									  |	(0x00 << 4)			//SPI MODE
									  | (0x00 << 6)			//CPOL=1
									  | (0x00 << 7)			//CPHA=1
									  | (0x07 << 8);		// the bit frequency is PCLK / (CPSDVSR * [SCR+1]).
	
	LPC_SSP1->CR1					=	(0X00 << 0)		    //正常操作
									  | (0X01 << 1)		    //spi 使能
									  | (0X00 << 2)		    //Master mode
									  | (0X00 << 3);	    //主机模式 不用关心
	LPC_SSP1->CPSR					=0X01;				    //PCLK 分频
	LPC_SSP1->ICR					=0X03;				    //中断清除
	LPC_SSP1->IMSC					=0x00;				    //禁止中断								
}
void	SPI_FLASH_Init(void)
{
	SPI_Flash_Config();
	GPIOSetDir( SPI_PORT, SPI_NSS, 1 );
	SPI_Flash_ReadWriteData(0x02);  	//使能状态寄存器中的写存储器
	SST25V_DBSY();
}
/**************************************************************************************************
函数名称： uint8_t	CV520_SPI_SendData(uint8_t Data)
函数功能： SPI 读写数据
入口参数： uint8_t Data
返回参数： 520 读出的数据
编写日期： 2013.8.13
程序作者： 夏立新
***************************************************************************************************/
uint8_t	SPI_Flash_ReadWriteData(uint8_t Data)
{	
	LPC_SSP1->DR	=Data;
	while(LPC_SSP1 ->SR & SPI_BSY);						//等待SPI传输为空
	return 	(LPC_SSP1->DR);	
}
/**************************************************************************************************
读取SPI_FLASH的状态寄存器
BIT7  6   5   4   3   2   1   0
SPR   RV  TB BP2 BP1 BP0 WEL BUSY
SPR:默认0,状态寄存器保护位,配合WP使用
TB,BP2,BP1,BP0:FLASH区域写保护设置
WEL:写使能锁定
BUSY:忙标记位(1,忙;0,空闲)
默认:0x00
***************************************************************************************************/
uint8_t SPI_Flash_ReadSR(void)  
{ 
		uint8_t byte=0; 
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData( SST25_ReadStatusReg );    //发送读取状态寄存器命令   
		byte=SPI_Flash_ReadWriteData(0Xff);                //读取一个字节
		FLASH_NSSHigh();     
		return byte;  
}

//写SPI_FLASH状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void SPI_FLASH_Write_SR(uint8_t sr)  
{  
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData(SST25_EnableWriteStatusReg);  //使能写状态寄存器命令                         
		SPI_Flash_ReadWriteData(SST25_WriteStatusReg);   	  //发送写取状态寄存器命令   
		SPI_Flash_ReadWriteData(sr);                          //写入一个字节               
		FLASH_NSSHigh(); 
} 
                            
//读取芯片ID SST25VF016的是 0XBF41
uint16_t SPI_Flash_ReadID(void)
{
		uint16_t Temp = 0;
		FLASH_NSSLow();      				                           
		SPI_Flash_ReadWriteData(0x90);					 	//发送读取ID命令90		        
		SPI_Flash_ReadWriteData(0x00);             		 	//发送24位的地址
		SPI_Flash_ReadWriteData(0x00);             
		SPI_Flash_ReadWriteData(0x00);		                              					   
		Temp	=SPI_Flash_ReadWriteData(0xFF)<<8;  	
		Temp   +=SPI_Flash_ReadWriteData(0xFF);
		FLASH_NSSHigh();                                     
		return Temp;
}
//读取SPI FLASH 
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535即64k)
void SPI_Flash_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)  
{
		uint16_t i;
		SPI_Flash_Wait_Busy();
		FLASH_NSSLow();                                                                                                                 
		SPI_Flash_ReadWriteData(SST25_ReadData);         		  //发送读取命令   
		SPI_Flash_ReadWriteData((uint8_t)((ReadAddr)>>16));   	  //发送24bit地址
		SPI_Flash_ReadWriteData((uint8_t)((ReadAddr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)ReadAddr);
		for(i=0;i<NumByteToRead;i++)
		{
		  *pBuffer++	=SPI_Flash_ReadWriteData(0XFF);   		  //循环读数 
		}
		FLASH_NSSHigh();            
} 
//地址自动增加的写数据A
void AutoAddressIncrement_WordProgramA(uint8_t Byte1, uint8_t Byte2, uint32_t Addr)
{
		SPI_FLASH_Write_Enable();
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData(SST25_AAI_WordProgram);				
		SPI_Flash_ReadWriteData((uint8_t)(Addr >> 16));		   //输入所要写数据的起始地址
		SPI_Flash_ReadWriteData((uint8_t)(Addr >> 8));
		SPI_Flash_ReadWriteData((uint8_t)(Addr & 0xFF));       
		SPI_Flash_ReadWriteData(Byte1);						   //发送最初的两个数据
		SPI_Flash_ReadWriteData(Byte2);
		FLASH_NSSHigh();
		SPI_Flash_Wait_Busy();
}

//地址自动增加的写数据B
void AutoAddressIncrement_WordProgramB(uint8_t state,uint8_t Byte1, uint8_t Byte2)
{
		SPI_FLASH_Write_Enable();
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData(SST25_AAI_WordProgram);
		SPI_Flash_ReadWriteData(Byte1);
		SPI_Flash_ReadWriteData(Byte2);
		FLASH_NSSHigh();
		SPI_Flash_Wait_Busy();
		if(state==1)
		{
		  SPI_FLASH_Write_Disable();
		}
		SPI_Flash_Wait_Busy();
}

//结合AB构成的地址自动增加的连续数据的写入
//具有先擦除待写区域的功能
//pBuffer：为待写数据组
//WriteAddr：所写数据的起始地址
//NumByteToWrite：所要写的数据的长度
void SPI_Flash_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
		uint16_t i,temp;
		uint32_t secpos;
		uint16_t secoff;
		uint16_t secremain;     
		//以下代码为擦除待写区域的代码
		secpos		=	WriteAddr	/	4096;      		//扇区（4K）地址0~511 for     SST25VF016
		secoff		=	WriteAddr	%	4096;             //在扇区内的偏移
		secremain	=	4096		-	secoff;         //扇区剩余空间大小
		if(NumByteToWrite<secremain)   temp=1;          //剩余空间大于所存数据
		else                                            //剩余空间小于所存数据
		{
		  i=NumByteToWrite-secremain;                   //判断还占了几个扇区
		  if(i%4096==0)	temp=i/4096+1;
		  else			temp=i/4096+2;
		}
		for(i=0;i<temp;i++)
		{
		  SPI_Flash_Erase_Sector((secpos+i)*4096);      //擦除将要写入数据的扇区   
		}
  //以下代码为将数据写入指定地址的代码
		if(NumByteToWrite%2==0)		temp=NumByteToWrite/2-1;
		else						temp=NumByteToWrite/2;
		AutoAddressIncrement_WordProgramA(pBuffer[0], pBuffer[1],WriteAddr );        //开始写数据
		for(i=1;i<temp;i++)	AutoAddressIncrement_WordProgramB(0,pBuffer[2*i], pBuffer[2*i+1]);
		if(NumByteToWrite%2==0)	AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-2], pBuffer[NumByteToWrite-1]);   //结束写数据
		else	AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-1],0);                                         //结束写数据

}
//写入1Byte数据
//pBuffer:待写的数据
//WriteAddr：待写数据的地址
void Flash_WriteByte(uint8_t* pBuffer,uint32_t WriteAddr)
{
		uint32_t secpos;
		secpos	=WriteAddr/4096;                                //扇区地址 0~511 for w25x16  4096=4k
		SPI_Flash_Erase_Sector(secpos);                         //擦除这个扇区
		SPI_FLASH_Write_Enable();                              	//SET WEL
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData(SST25_ByteProgram );       		//发送写页命令		  
		SPI_Flash_ReadWriteData((uint8_t)((WriteAddr)>>16));  	//发送24bit地址 
		SPI_Flash_ReadWriteData((uint8_t)((WriteAddr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)WriteAddr);                 
		SPI_Flash_ReadWriteData(pBuffer[0]);					//发送待写的数据
		FLASH_NSSHigh();
		SPI_Flash_Wait_Busy();                                  //等待写完
}

//擦除整个芯片
//整片擦除时间:
//W25X16:25s
//W25X32:40s
//W25X64:40s
//等待时间超长...
void SPI_Flash_Erase_Chip(void)  
{                                            
		SPI_FLASH_Write_Enable();                          	//SET WEL
		SPI_Flash_Wait_Busy(); 
		FLASH_NSSLow(); 
		SPI_Flash_ReadWriteData(SST25_ChipErase);       	//发送片擦除命令
		FLASH_NSSHigh();
		SPI_Flash_Wait_Busy();                              //等待写完               
}
//擦除一个扇区
//Dst_Addr:扇区地址 0~511 for w25x16
//擦除一个山区的最少时间:150ms
void SPI_Flash_Erase_Sector(uint32_t Dst_Addr)  
{  
		SPI_FLASH_Write_Enable();                             	//SET WEL           
		SPI_Flash_Wait_Busy();
		FLASH_NSSLow();  
		SPI_Flash_ReadWriteData(SST25_4KByte_BlockERASE);       //发送扇区擦除指令
		SPI_Flash_ReadWriteData((uint8_t)((Dst_Addr)>>16));     //发送24bit地址   
		SPI_Flash_ReadWriteData((uint8_t)((Dst_Addr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)Dst_Addr);
		FLASH_NSSHigh();              
		SPI_Flash_Wait_Busy();                                  //等待擦除完
} 
//等待空闲
void	SST25V_EBSY(void)
{		 
		 FLASH_NSSLow(); 
		 SPI_Flash_ReadWriteData( SST25_EBSY);
		 FLASH_NSSHigh();
}
void	SST25V_DBSY(void)
{		 
		 FLASH_NSSLow(); 
		 SPI_Flash_ReadWriteData( SST25_DBSY);
		 FLASH_NSSHigh();
}
void	SPI_FLASH_Write_Enable(void)
{		 
		 FLASH_NSSLow(); 
		 SPI_Flash_ReadWriteData( SST25_WriteEnable);
		 FLASH_NSSHigh();
}
void	SPI_FLASH_Write_Disable(void)
{		 
		 FLASH_NSSLow(); 
		 SPI_Flash_ReadWriteData( SST25_WriteDisable);
		 FLASH_NSSHigh();
}






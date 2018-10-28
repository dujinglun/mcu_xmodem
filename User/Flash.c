#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include	"Flash.h"
#include	"uart.h"	 
/**************************************************************
函数名称： void	 SPI_Init(void)
函数功能： SPI初始化
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	软件片选
**************************************************************/
void	 SPI_Init(void)
{
	LPC_IOCON->PIO1_20			&=~0x07;  	    //SCK1
	LPC_IOCON->PIO1_21			&=~0x07;  	    //MISO
	LPC_IOCON->PIO1_22			&=~0x07;  	    //MOSI
	LPC_IOCON->PIO1_20			|=0x02;  	    //SCK1
	LPC_IOCON->PIO1_21			|=0x02;  	    //MISO
	LPC_IOCON->PIO1_22			|=0x02;  	    //MOSI									  									   	
	LPC_SYSCON->PRESETCTRL 		|=(0x01 << 2);	//禁止SSP1复位
	LPC_SYSCON->SYSAHBCLKCTRL 	|=(0X01 << 18);	//SPI1 时钟开启
	LPC_SYSCON->SSP1CLKDIV		=0x02;			//SPI1 时钟分配
	LPC_SSP1->CR0				= (0x07 << 0)	//8-BIT
								| (0x00 << 4)	//SPI MODE
								| (0x00 << 6)	//CPOL=1
								| (0x00 << 7)	//CPHA=1
								| (0x07 << 8);  // the bit frequency is PCLK / (CPSDVSR * [SCR+1]).	
	LPC_SSP1->CR1				= (0X00 << 0)	//正常操作
								| (0X01 << 1)   //spi 使能
								| (0X00 << 2)   //Master mode
								| (0X00 << 3);	//主机模式 不用关心
	LPC_SSP1->CPSR				=0x02;		//PCLK 分频
	LPC_SSP1->ICR				=0X03;		//中断清除
	LPC_SSP1->IMSC				=0x00;		//禁止中断
}
/**************************************************************
函数名称： uint8_t	SPI_TransByte(uint8_t Byte)
函数功能： SPI收发1个字节数据
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
uint8_t	SPI_TransByte(uint8_t Byte)
{
	LPC_SSP1->DR	=Byte;
	while(LPC_SSP1 ->SR & SPI_BSY);		//等待SPI传输为空
	return 	(LPC_SSP1->DR);	
}
/**************************************************************
函数名称： uint8_t	FLASH_ReadSR(void)
函数功能： 读FLASH状态字节
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
uint8_t	FLASH_ReadSR(void)
{
	uint8_t byte=0; 
	FLASH_CSL();
	SPI_TransByte( FLASH_RDSR );    //发送读取状态寄存器命令   
	byte=SPI_TransByte(0Xff);       //读取一个字节
	FLASH_CSH();     
	return byte; 
}
/**************************************************************
函数名称： void	FLASH_OpEBSY(void)
函数功能： 使能AAI编程时BUSY输出
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_OpEBSY(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_EBSY );    	//发送读取状态寄存器命令   
	FLASH_CSH();
}
#endif
/**************************************************************
函数名称： void	FLASH_OpDBSY(void)
函数功能： 关闭AAI编程时BUSY输出
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_OpDBSY(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_DBSY );    	//发送读取状态寄存器命令   
	FLASH_CSH();
}
#endif
/**************************************************************
函数名称： void	FLASH_OpWREN(void)
函数功能： 写数据使能操作
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_OpWREN(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_WREN );    	//发送读取状态寄存器命令   
	FLASH_CSH();
}
/**************************************************************
函数名称： void	FLASH_OpEWSR(void)
函数功能： 写数据使能操作
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_OpEWSR(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_EWSR );    	//发送读取状态寄存器命令   
	FLASH_CSH();
}
/**************************************************************
函数名称： void	FLASH_OpWREN(void)
函数功能： 写数据失能操作
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_OpWRDI(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_WRDI );    	//发送读取状态寄存器命令   
	FLASH_CSH();
}
/**************************************************************
函数名称： void	FLASH_IsFree(void)
函数功能： 等待FLASH编程结束
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_IsFree(void)
{
	  while(FLASH_ReadSR() & 0x01);
}
/**************************************************************
函数名称： void	FLASH_Init(void)
函数功能： FLASH 初始化
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_Init(void)
{
	SPI_Init();
	GPIOSetDir( SPI_PORT, SPI_NSS, 1 );
	SPI_TransByte(0x02);  			//使能状态寄存器中的写存储器
	FLASH_CSL();
	SPI_TransByte( FLASH_DBSY );    //发送读取状态寄存器命令   
	FLASH_CSH();
}
/**************************************************************
函数名称： void	FLASH_WriteSR(uint8_t New)
函数功能： 写状态寄存器
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：
BIT0	BIT1	BIT2	BIT3	BIT4	BIT5	BIT6	BIT7
BUSY	WEL		 BP0	BP1		BP2		BP3		AAI		BPL
仅仅只有  BP0 	BP1 	BP2 	BP3 	BPL可写，其他只读
**************************************************************/
void	FLASH_WriteSR(uint8_t New)
{
	FLASH_CSL();
	SPI_TransByte(FLASH_EWSR);  	//使能写状态寄存器命令                         
	SPI_TransByte(FLASH_WRSR);   	//发送写取状态寄存器命令   
	SPI_TransByte(New);             //写入一个字节               
	FLASH_CSH();	
}

void	FLASH_ProtectALL(void)
{
	FLASH_IsFree();
	FLASH_WriteSR(0x3c);			
}
/**************************************************************
函数名称： uint16_t	FLASH_ReadID(void)
函数功能： 读FLASH_ID号
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
uint16_t	FLASH_ReadID(void)
{
	uint16_t Temp = 0;
	FLASH_CSL();      				                           
	SPI_TransByte(FLASH_RDID);		//发送读取ID命令90		        
	SPI_TransByte(0x00);            //发送24位的地址
	SPI_TransByte(0x00);             
	SPI_TransByte(0x00);		                              					   
	Temp	=SPI_TransByte(0xFF)<<8;  	
	Temp   |=SPI_TransByte(0xFF);
	FLASH_CSH();                                     
	return Temp;	
}
/**************************************************************
函数名称： uint16_t	FLASH_ReadID(void)
函数功能： 读FLASH_ID号
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
uint16_t	FLASH_ReadJEDEC_ID(void)
{
	uint16_t Temp = 0;
	FLASH_CSL();      				                           
	SPI_TransByte(FLASH_JEDEC_ID);		//发送读取ID命令90
	SPI_TransByte(0xFF);		        		                              					   
	Temp	=SPI_TransByte(0xFF)<<8;  	
	Temp   |=SPI_TransByte(0xFF);
	FLASH_CSH();
	return Temp;                                     	
}
/**************************************************************
函数名称： void	FLASH_OpChipErase(void)
函数功能： 芯片擦除
入口参数： 
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_OpChipErase(void)
{
	FLASH_OpWREN();
	FLASH_IsFree();
	FLASH_CSL();
	SPI_TransByte(FLASH_ChipErase);
	FLASH_CSH();
	FLASH_IsFree();	
}
/**************************************************************
函数名称： void	FLASH_OpSectorErase(uint32_t Addr)
函数功能： 芯片扇区擦除
入口参数： 扇区入口地址		4096的倍数
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_OpSectorErase(uint32_t Addr)
{
	FLASH_OpWREN();
	FLASH_IsFree();
	FLASH_CSL();
	SPI_TransByte(FLASH_4KSectorErase);
	SPI_TransByte((uint8_t)((Addr)>>16));     //发送24bit地址
	SPI_TransByte((uint8_t)((Addr)>>8));  
	SPI_TransByte((uint8_t)Addr);
	FLASH_CSH();
	FLASH_IsFree();
}
/**************************************************************
函数名称： void	FLASH_ReadBytes(uint8_t *pBuff,uint32_t Addr,uint16_t ReadLength)
函数功能： 多字节读
入口参数： (uint8_t *pBuff,		数据缓存地址
			uint32_t Addr,		读FLASH的起始地址
			uint16_t ReadLength)读字节个数
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_ReadBytes(uint8_t *pBuff,uint32_t Addr,uint16_t ReadLength)
{
	FLASH_IsFree();
	FLASH_CSL();                                                                                                                 
	SPI_TransByte(FLASH_Read);         		  //发送读取命令   
	SPI_TransByte((uint8_t)((Addr)>>16));     //发送24bit地址
	SPI_TransByte((uint8_t)((Addr)>>8));  
	SPI_TransByte((uint8_t)Addr);
	while(ReadLength--)
	{
	  *pBuff++	=SPI_TransByte(0XFF);   	  //循环读数 
	}
	FLASH_CSH(); 	
}
/**************************************************************
函数名称： void	FLASH_AAIWPMA(uint8_t Byte1,uint8_t Byte2,uint32_t Addr)
函数功能： WORD编程模式A
入口参数： (uint8_t Byte1,
			uint8_t Byte2,
			uint32_t Addr)
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_AAIWPMA(uint8_t Byte1,uint8_t Byte2,uint32_t Addr)
{
	FLASH_CSL();
	SPI_TransByte(FLASH_AAIWProgram);				
	SPI_TransByte((uint8_t)(Addr >> 16));		   //输入所要写数据的起始地址
	SPI_TransByte((uint8_t)(Addr >> 8));
	SPI_TransByte((uint8_t)(Addr));       
	SPI_TransByte(Byte1);						   //发送最初的两个数据
	SPI_TransByte(Byte2);
	FLASH_CSH();
	FLASH_IsFree();
}
#endif
/**************************************************************
函数名称： void	FLASH_AAIWPMB(uint8_t Byte1,uint8_t Byte2)
函数功能： WORD编程模式B
入口参数： (uint8_t Byte1,
			uint8_t Byte2,
		    )
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_AAIWPMB(uint8_t Byte1,uint8_t Byte2)
{
	FLASH_CSL();
	SPI_TransByte(FLASH_AAIWProgram);
	SPI_TransByte(Byte1);
	SPI_TransByte(Byte2);
	FLASH_CSH();
	FLASH_IsFree();
}
#endif
/**************************************************************
函数名称： void	FLASH_WriteBytes(uint8_t *pBuff,uint32_t Addr,uint16_t Bytes)
函数功能： 多字节写
入口参数： (uint8_t *pBuff,
			uint32_t Addr,
			uint16_t Bytes)
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_WriteBytes(uint8_t *pBuff,uint32_t Addr,uint16_t Bytes)
{
		uint16_t i,temp;
		uint32_t secpos;
		uint16_t secoff;
		uint16_t secremain;     
		//以下代码为擦除待写区域的代码
		secpos		=	Addr	/	4096;      		  //扇区（4K）地址0~511 for     SST25VF016
		secoff		=	Addr	%	4096;             //在扇区内的偏移
		secremain	=	4096	-	secoff;       	  //扇区剩余空间大小
		if(Bytes <secremain)   temp=1;                //剩余空间大于所存数据
		else                                          //剩余空间小于所存数据
		{
		  i= Bytes - secremain;                       //判断还占了几个扇区
		  if(i%4096==0)	temp=i/4096+1;
		  else			temp=i/4096+2;
		}
		for(i=0;i<temp;i++)
		{
		  FLASH_OpSectorErase((secpos+i)*4096);       //擦除将要写入数据的扇区   
		}
  //以下代码为将数据写入指定地址的代码
		temp=Bytes >> 1;
		FLASH_OpWREN();
		FLASH_AAIWPMA(pBuff[0], pBuff[1],Addr );     //开始写数据
		for(i=1;i<temp;i++)	
		{
		   FLASH_AAIWPMB(pBuff[2*i], pBuff[2*i+1]);
		}
		if(Bytes & 0x01)
		{
		  FLASH_AAIWPMB(pBuff[Bytes-1],0);
		}
		FLASH_OpWRDI();
	  	FLASH_IsFree(); 
}
#endif
/**************************************************************
函数名称： void	FLASH_WriteByte(uint8_t Byte,uint32_t Addr,uint8_t EraseEn)
函数功能： 单字节写
入口参数： (uint8_t *pBuff,
			uint32_t Addr,
			uint16_t Bytes)
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：	
**************************************************************/
void	FLASH_WriteByte(uint8_t Byte,uint32_t Addr,uint8_t EraseEn)
{                            
	if( EraseEn == 0XAA)
	{
		FLASH_OpSectorErase( Addr/4096 );  //擦除这个扇区	//扇区地址 0~511 for w25x16  4096=4k	
	}	
	FLASH_OpWREN();                              
	FLASH_CSL();
	SPI_TransByte(FLASH_ByteProgram );      //发送写页命令		  
	SPI_TransByte((uint8_t)((Addr)>>16));  	//发送24bit地址 
	SPI_TransByte((uint8_t)((Addr)>>8));  
	SPI_TransByte((uint8_t)Addr);                 
	SPI_TransByte( Byte );					//发送待写的数据
	FLASH_CSH();
	FLASH_OpWRDI();
	FLASH_IsFree();    
}
/**************************************************************
函数名称： void	FLASH_WriteByte(uint8_t Byte,uint32_t Addr,uint8_t EraseEn)
函数功能： 单字节写
入口参数： (uint8_t *pBuff,
			uint32_t Addr,
			uint16_t Bytes)
返回参数： 
编写日期： 2013.9.5
程序作者： 夏立新	 ZeroOne :631640356
说明：
BIT0	BIT1	BIT2	BIT3	BIT4	BIT5	BIT6	BIT7
BUSY	WEL		 BP0	BP1		BP2		BP3		AAI		BPL	
**************************************************************/
void	FLASH_Test(void)
{
	 uint32_t	Temp ,ErrorCnt=0;
	 uint8_t	ReadValue ,i;
	 uint8_t	Table[]="6.存储器校验中,请耐心等待...\r\n";
	 print_string("-------------SPI存储器测试中-----------\r\n");
	 Myprintf("1.存储器的ID号是: ", FLASH_ReadID());			   
	 Myprintf("2.存储器特征的ID号是: ", FLASH_ReadJEDEC_ID());		//？？？？？？？？
	 print_string("3.存储器擦除开始...\r\n");
	 FLASH_OpChipErase();
	 print_string("4.存储器擦除结束...\r\n");
	 print_string("5.存储器扇区校验开始...\r\n");
	 ErrorCnt	= sizeof(Table) + 1024;
	 for(i=0,Temp=1024;Temp<ErrorCnt;Temp++,i++)
	 {
	 	 FLASH_WriteByte(Table[i] ,Temp,0);
		 Table[i]=0x00;
	 }
	 for(i=0,Temp=1024;Temp<ErrorCnt;Temp++,i++)
	 {
	 	 FLASH_ReadBytes(Table+i,Temp,1);
	 }
	 print_string(Table);
	 for(Temp=4096;Temp<0xFFFFF;Temp++)
	 {
	 	FLASH_WriteByte(Temp & 0xff,Temp,0);
	 }
	 ErrorCnt	=0x00;
	 for(Temp=4096;Temp<0xFFFFF;Temp++)
	 {
	 	FLASH_ReadBytes(&ReadValue,Temp,1);
		if(ReadValue !=(Temp & 0xff))
		{
			ErrorCnt++;
		}
	 }
	 Myprintf("7.存储器校验结束,坏字节数: ", ErrorCnt);
	 if(ErrorCnt) 	  print_string("8.该存储器不合格，不能使用...\r\n");
	 else			  print_string("9.该存储器合格，可以投入使用...\r\n");
	 print_string("-------------SPI存储器测试结束-----------\r\n");
}													   


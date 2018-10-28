#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include	"SPI_Flash.h"

/**************************************************************************************************
�������ƣ� void	CV520_SPI_Config(void)
�������ܣ� LPC11UXX SP1 ����
��ڲ����� ��
���ز����� ��
��д���ڣ� 2013.8.13
�������ߣ� ������
***************************************************************************************************/
void	SPI_Flash_Config(void)
{
	LPC_IOCON->PIO1_20				&=~0x07;  	    //SCK1
	LPC_IOCON->PIO1_21				&=~0x07;  	    //MISO
	LPC_IOCON->PIO1_22				&=~0x07;  	    //MOSI

	LPC_IOCON->PIO1_20				|=0x02;  	    //SCK1
	LPC_IOCON->PIO1_21				|=0x02;  	    //MISO
	LPC_IOCON->PIO1_22				|=0x02;  	    //MOSI
									  									   	
	LPC_SYSCON->PRESETCTRL 			|=(0x01 << 2);			//��ֹSSP1��λ
	LPC_SYSCON->SYSAHBCLKCTRL 		|=(0X01 << 18);			//SPI1 ʱ�ӿ���
	LPC_SYSCON->SSP1CLKDIV			=0x02;					//SPI1 ʱ�ӷ���
	LPC_SSP1->CR0					=	(0x07 << 0)	   		//8-BIT
									  |	(0x00 << 4)			//SPI MODE
									  | (0x00 << 6)			//CPOL=1
									  | (0x00 << 7)			//CPHA=1
									  | (0x07 << 8);		// the bit frequency is PCLK / (CPSDVSR * [SCR+1]).
	
	LPC_SSP1->CR1					=	(0X00 << 0)		    //��������
									  | (0X01 << 1)		    //spi ʹ��
									  | (0X00 << 2)		    //Master mode
									  | (0X00 << 3);	    //����ģʽ ���ù���
	LPC_SSP1->CPSR					=0X01;				    //PCLK ��Ƶ
	LPC_SSP1->ICR					=0X03;				    //�ж����
	LPC_SSP1->IMSC					=0x00;				    //��ֹ�ж�								
}
void	SPI_FLASH_Init(void)
{
	SPI_Flash_Config();
	GPIOSetDir( SPI_PORT, SPI_NSS, 1 );
	SPI_Flash_ReadWriteData(0x02);  	//ʹ��״̬�Ĵ����е�д�洢��
	SST25V_DBSY();
}
/**************************************************************************************************
�������ƣ� uint8_t	CV520_SPI_SendData(uint8_t Data)
�������ܣ� SPI ��д����
��ڲ����� uint8_t Data
���ز����� 520 ����������
��д���ڣ� 2013.8.13
�������ߣ� ������
***************************************************************************************************/
uint8_t	SPI_Flash_ReadWriteData(uint8_t Data)
{	
	LPC_SSP1->DR	=Data;
	while(LPC_SSP1 ->SR & SPI_BSY);						//�ȴ�SPI����Ϊ��
	return 	(LPC_SSP1->DR);	
}
/**************************************************************************************************
��ȡSPI_FLASH��״̬�Ĵ���
BIT7  6   5   4   3   2   1   0
SPR   RV  TB BP2 BP1 BP0 WEL BUSY
SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
TB,BP2,BP1,BP0:FLASH����д��������
WEL:дʹ������
BUSY:æ���λ(1,æ;0,����)
Ĭ��:0x00
***************************************************************************************************/
uint8_t SPI_Flash_ReadSR(void)  
{ 
		uint8_t byte=0; 
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData( SST25_ReadStatusReg );    //���Ͷ�ȡ״̬�Ĵ�������   
		byte=SPI_Flash_ReadWriteData(0Xff);                //��ȡһ���ֽ�
		FLASH_NSSHigh();     
		return byte;  
}

//дSPI_FLASH״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void SPI_FLASH_Write_SR(uint8_t sr)  
{  
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData(SST25_EnableWriteStatusReg);  //ʹ��д״̬�Ĵ�������                         
		SPI_Flash_ReadWriteData(SST25_WriteStatusReg);   	  //����дȡ״̬�Ĵ�������   
		SPI_Flash_ReadWriteData(sr);                          //д��һ���ֽ�               
		FLASH_NSSHigh(); 
} 
                            
//��ȡоƬID SST25VF016���� 0XBF41
uint16_t SPI_Flash_ReadID(void)
{
		uint16_t Temp = 0;
		FLASH_NSSLow();      				                           
		SPI_Flash_ReadWriteData(0x90);					 	//���Ͷ�ȡID����90		        
		SPI_Flash_ReadWriteData(0x00);             		 	//����24λ�ĵ�ַ
		SPI_Flash_ReadWriteData(0x00);             
		SPI_Flash_ReadWriteData(0x00);		                              					   
		Temp	=SPI_Flash_ReadWriteData(0xFF)<<8;  	
		Temp   +=SPI_Flash_ReadWriteData(0xFF);
		FLASH_NSSHigh();                                     
		return Temp;
}
//��ȡSPI FLASH 
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535��64k)
void SPI_Flash_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)  
{
		uint16_t i;
		SPI_Flash_Wait_Busy();
		FLASH_NSSLow();                                                                                                                 
		SPI_Flash_ReadWriteData(SST25_ReadData);         		  //���Ͷ�ȡ����   
		SPI_Flash_ReadWriteData((uint8_t)((ReadAddr)>>16));   	  //����24bit��ַ
		SPI_Flash_ReadWriteData((uint8_t)((ReadAddr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)ReadAddr);
		for(i=0;i<NumByteToRead;i++)
		{
		  *pBuffer++	=SPI_Flash_ReadWriteData(0XFF);   		  //ѭ������ 
		}
		FLASH_NSSHigh();            
} 
//��ַ�Զ����ӵ�д����A
void AutoAddressIncrement_WordProgramA(uint8_t Byte1, uint8_t Byte2, uint32_t Addr)
{
		SPI_FLASH_Write_Enable();
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData(SST25_AAI_WordProgram);				
		SPI_Flash_ReadWriteData((uint8_t)(Addr >> 16));		   //������Ҫд���ݵ���ʼ��ַ
		SPI_Flash_ReadWriteData((uint8_t)(Addr >> 8));
		SPI_Flash_ReadWriteData((uint8_t)(Addr & 0xFF));       
		SPI_Flash_ReadWriteData(Byte1);						   //�����������������
		SPI_Flash_ReadWriteData(Byte2);
		FLASH_NSSHigh();
		SPI_Flash_Wait_Busy();
}

//��ַ�Զ����ӵ�д����B
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

//���AB���ɵĵ�ַ�Զ����ӵ��������ݵ�д��
//�����Ȳ�����д����Ĺ���
//pBuffer��Ϊ��д������
//WriteAddr����д���ݵ���ʼ��ַ
//NumByteToWrite����Ҫд�����ݵĳ���
void SPI_Flash_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
		uint16_t i,temp;
		uint32_t secpos;
		uint16_t secoff;
		uint16_t secremain;     
		//���´���Ϊ������д����Ĵ���
		secpos		=	WriteAddr	/	4096;      		//������4K����ַ0~511 for     SST25VF016
		secoff		=	WriteAddr	%	4096;             //�������ڵ�ƫ��
		secremain	=	4096		-	secoff;         //����ʣ��ռ��С
		if(NumByteToWrite<secremain)   temp=1;          //ʣ��ռ������������
		else                                            //ʣ��ռ�С����������
		{
		  i=NumByteToWrite-secremain;                   //�жϻ�ռ�˼�������
		  if(i%4096==0)	temp=i/4096+1;
		  else			temp=i/4096+2;
		}
		for(i=0;i<temp;i++)
		{
		  SPI_Flash_Erase_Sector((secpos+i)*4096);      //������Ҫд�����ݵ�����   
		}
  //���´���Ϊ������д��ָ����ַ�Ĵ���
		if(NumByteToWrite%2==0)		temp=NumByteToWrite/2-1;
		else						temp=NumByteToWrite/2;
		AutoAddressIncrement_WordProgramA(pBuffer[0], pBuffer[1],WriteAddr );        //��ʼд����
		for(i=1;i<temp;i++)	AutoAddressIncrement_WordProgramB(0,pBuffer[2*i], pBuffer[2*i+1]);
		if(NumByteToWrite%2==0)	AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-2], pBuffer[NumByteToWrite-1]);   //����д����
		else	AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-1],0);                                         //����д����

}
//д��1Byte����
//pBuffer:��д������
//WriteAddr����д���ݵĵ�ַ
void Flash_WriteByte(uint8_t* pBuffer,uint32_t WriteAddr)
{
		uint32_t secpos;
		secpos	=WriteAddr/4096;                                //������ַ 0~511 for w25x16  4096=4k
		SPI_Flash_Erase_Sector(secpos);                         //�����������
		SPI_FLASH_Write_Enable();                              	//SET WEL
		FLASH_NSSLow();
		SPI_Flash_ReadWriteData(SST25_ByteProgram );       		//����дҳ����		  
		SPI_Flash_ReadWriteData((uint8_t)((WriteAddr)>>16));  	//����24bit��ַ 
		SPI_Flash_ReadWriteData((uint8_t)((WriteAddr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)WriteAddr);                 
		SPI_Flash_ReadWriteData(pBuffer[0]);					//���ʹ�д������
		FLASH_NSSHigh();
		SPI_Flash_Wait_Busy();                                  //�ȴ�д��
}

//��������оƬ
//��Ƭ����ʱ��:
//W25X16:25s
//W25X32:40s
//W25X64:40s
//�ȴ�ʱ�䳬��...
void SPI_Flash_Erase_Chip(void)  
{                                            
		SPI_FLASH_Write_Enable();                          	//SET WEL
		SPI_Flash_Wait_Busy(); 
		FLASH_NSSLow(); 
		SPI_Flash_ReadWriteData(SST25_ChipErase);       	//����Ƭ��������
		FLASH_NSSHigh();
		SPI_Flash_Wait_Busy();                              //�ȴ�д��               
}
//����һ������
//Dst_Addr:������ַ 0~511 for w25x16
//����һ��ɽ��������ʱ��:150ms
void SPI_Flash_Erase_Sector(uint32_t Dst_Addr)  
{  
		SPI_FLASH_Write_Enable();                             	//SET WEL           
		SPI_Flash_Wait_Busy();
		FLASH_NSSLow();  
		SPI_Flash_ReadWriteData(SST25_4KByte_BlockERASE);       //������������ָ��
		SPI_Flash_ReadWriteData((uint8_t)((Dst_Addr)>>16));     //����24bit��ַ   
		SPI_Flash_ReadWriteData((uint8_t)((Dst_Addr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)Dst_Addr);
		FLASH_NSSHigh();              
		SPI_Flash_Wait_Busy();                                  //�ȴ�������
} 
//�ȴ�����
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






#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include	"Flash.h"
#include	"uart.h"	 
/**************************************************************
�������ƣ� void	 SPI_Init(void)
�������ܣ� SPI��ʼ��
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	���Ƭѡ
**************************************************************/
void	 SPI_Init(void)
{
	LPC_IOCON->PIO1_20			&=~0x07;  	    //SCK1
	LPC_IOCON->PIO1_21			&=~0x07;  	    //MISO
	LPC_IOCON->PIO1_22			&=~0x07;  	    //MOSI
	LPC_IOCON->PIO1_20			|=0x02;  	    //SCK1
	LPC_IOCON->PIO1_21			|=0x02;  	    //MISO
	LPC_IOCON->PIO1_22			|=0x02;  	    //MOSI									  									   	
	LPC_SYSCON->PRESETCTRL 		|=(0x01 << 2);	//��ֹSSP1��λ
	LPC_SYSCON->SYSAHBCLKCTRL 	|=(0X01 << 18);	//SPI1 ʱ�ӿ���
	LPC_SYSCON->SSP1CLKDIV		=0x02;			//SPI1 ʱ�ӷ���
	LPC_SSP1->CR0				= (0x07 << 0)	//8-BIT
								| (0x00 << 4)	//SPI MODE
								| (0x00 << 6)	//CPOL=1
								| (0x00 << 7)	//CPHA=1
								| (0x07 << 8);  // the bit frequency is PCLK / (CPSDVSR * [SCR+1]).	
	LPC_SSP1->CR1				= (0X00 << 0)	//��������
								| (0X01 << 1)   //spi ʹ��
								| (0X00 << 2)   //Master mode
								| (0X00 << 3);	//����ģʽ ���ù���
	LPC_SSP1->CPSR				=0x02;		//PCLK ��Ƶ
	LPC_SSP1->ICR				=0X03;		//�ж����
	LPC_SSP1->IMSC				=0x00;		//��ֹ�ж�
}
/**************************************************************
�������ƣ� uint8_t	SPI_TransByte(uint8_t Byte)
�������ܣ� SPI�շ�1���ֽ�����
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
uint8_t	SPI_TransByte(uint8_t Byte)
{
	LPC_SSP1->DR	=Byte;
	while(LPC_SSP1 ->SR & SPI_BSY);		//�ȴ�SPI����Ϊ��
	return 	(LPC_SSP1->DR);	
}
/**************************************************************
�������ƣ� uint8_t	FLASH_ReadSR(void)
�������ܣ� ��FLASH״̬�ֽ�
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
uint8_t	FLASH_ReadSR(void)
{
	uint8_t byte=0; 
	FLASH_CSL();
	SPI_TransByte( FLASH_RDSR );    //���Ͷ�ȡ״̬�Ĵ�������   
	byte=SPI_TransByte(0Xff);       //��ȡһ���ֽ�
	FLASH_CSH();     
	return byte; 
}
/**************************************************************
�������ƣ� void	FLASH_OpEBSY(void)
�������ܣ� ʹ��AAI���ʱBUSY���
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_OpEBSY(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_EBSY );    	//���Ͷ�ȡ״̬�Ĵ�������   
	FLASH_CSH();
}
#endif
/**************************************************************
�������ƣ� void	FLASH_OpDBSY(void)
�������ܣ� �ر�AAI���ʱBUSY���
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_OpDBSY(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_DBSY );    	//���Ͷ�ȡ״̬�Ĵ�������   
	FLASH_CSH();
}
#endif
/**************************************************************
�������ƣ� void	FLASH_OpWREN(void)
�������ܣ� д����ʹ�ܲ���
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_OpWREN(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_WREN );    	//���Ͷ�ȡ״̬�Ĵ�������   
	FLASH_CSH();
}
/**************************************************************
�������ƣ� void	FLASH_OpEWSR(void)
�������ܣ� д����ʹ�ܲ���
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_OpEWSR(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_EWSR );    	//���Ͷ�ȡ״̬�Ĵ�������   
	FLASH_CSH();
}
/**************************************************************
�������ƣ� void	FLASH_OpWREN(void)
�������ܣ� д����ʧ�ܲ���
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_OpWRDI(void)
{
	FLASH_CSL();
	SPI_TransByte( FLASH_WRDI );    	//���Ͷ�ȡ״̬�Ĵ�������   
	FLASH_CSH();
}
/**************************************************************
�������ƣ� void	FLASH_IsFree(void)
�������ܣ� �ȴ�FLASH��̽���
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_IsFree(void)
{
	  while(FLASH_ReadSR() & 0x01);
}
/**************************************************************
�������ƣ� void	FLASH_Init(void)
�������ܣ� FLASH ��ʼ��
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_Init(void)
{
	SPI_Init();
	GPIOSetDir( SPI_PORT, SPI_NSS, 1 );
	SPI_TransByte(0x02);  			//ʹ��״̬�Ĵ����е�д�洢��
	FLASH_CSL();
	SPI_TransByte( FLASH_DBSY );    //���Ͷ�ȡ״̬�Ĵ�������   
	FLASH_CSH();
}
/**************************************************************
�������ƣ� void	FLASH_WriteSR(uint8_t New)
�������ܣ� д״̬�Ĵ���
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����
BIT0	BIT1	BIT2	BIT3	BIT4	BIT5	BIT6	BIT7
BUSY	WEL		 BP0	BP1		BP2		BP3		AAI		BPL
����ֻ��  BP0 	BP1 	BP2 	BP3 	BPL��д������ֻ��
**************************************************************/
void	FLASH_WriteSR(uint8_t New)
{
	FLASH_CSL();
	SPI_TransByte(FLASH_EWSR);  	//ʹ��д״̬�Ĵ�������                         
	SPI_TransByte(FLASH_WRSR);   	//����дȡ״̬�Ĵ�������   
	SPI_TransByte(New);             //д��һ���ֽ�               
	FLASH_CSH();	
}

void	FLASH_ProtectALL(void)
{
	FLASH_IsFree();
	FLASH_WriteSR(0x3c);			
}
/**************************************************************
�������ƣ� uint16_t	FLASH_ReadID(void)
�������ܣ� ��FLASH_ID��
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
uint16_t	FLASH_ReadID(void)
{
	uint16_t Temp = 0;
	FLASH_CSL();      				                           
	SPI_TransByte(FLASH_RDID);		//���Ͷ�ȡID����90		        
	SPI_TransByte(0x00);            //����24λ�ĵ�ַ
	SPI_TransByte(0x00);             
	SPI_TransByte(0x00);		                              					   
	Temp	=SPI_TransByte(0xFF)<<8;  	
	Temp   |=SPI_TransByte(0xFF);
	FLASH_CSH();                                     
	return Temp;	
}
/**************************************************************
�������ƣ� uint16_t	FLASH_ReadID(void)
�������ܣ� ��FLASH_ID��
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
uint16_t	FLASH_ReadJEDEC_ID(void)
{
	uint16_t Temp = 0;
	FLASH_CSL();      				                           
	SPI_TransByte(FLASH_JEDEC_ID);		//���Ͷ�ȡID����90
	SPI_TransByte(0xFF);		        		                              					   
	Temp	=SPI_TransByte(0xFF)<<8;  	
	Temp   |=SPI_TransByte(0xFF);
	FLASH_CSH();
	return Temp;                                     	
}
/**************************************************************
�������ƣ� void	FLASH_OpChipErase(void)
�������ܣ� оƬ����
��ڲ����� 
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
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
�������ƣ� void	FLASH_OpSectorErase(uint32_t Addr)
�������ܣ� оƬ��������
��ڲ����� ������ڵ�ַ		4096�ı���
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_OpSectorErase(uint32_t Addr)
{
	FLASH_OpWREN();
	FLASH_IsFree();
	FLASH_CSL();
	SPI_TransByte(FLASH_4KSectorErase);
	SPI_TransByte((uint8_t)((Addr)>>16));     //����24bit��ַ
	SPI_TransByte((uint8_t)((Addr)>>8));  
	SPI_TransByte((uint8_t)Addr);
	FLASH_CSH();
	FLASH_IsFree();
}
/**************************************************************
�������ƣ� void	FLASH_ReadBytes(uint8_t *pBuff,uint32_t Addr,uint16_t ReadLength)
�������ܣ� ���ֽڶ�
��ڲ����� (uint8_t *pBuff,		���ݻ����ַ
			uint32_t Addr,		��FLASH����ʼ��ַ
			uint16_t ReadLength)���ֽڸ���
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_ReadBytes(uint8_t *pBuff,uint32_t Addr,uint16_t ReadLength)
{
	FLASH_IsFree();
	FLASH_CSL();                                                                                                                 
	SPI_TransByte(FLASH_Read);         		  //���Ͷ�ȡ����   
	SPI_TransByte((uint8_t)((Addr)>>16));     //����24bit��ַ
	SPI_TransByte((uint8_t)((Addr)>>8));  
	SPI_TransByte((uint8_t)Addr);
	while(ReadLength--)
	{
	  *pBuff++	=SPI_TransByte(0XFF);   	  //ѭ������ 
	}
	FLASH_CSH(); 	
}
/**************************************************************
�������ƣ� void	FLASH_AAIWPMA(uint8_t Byte1,uint8_t Byte2,uint32_t Addr)
�������ܣ� WORD���ģʽA
��ڲ����� (uint8_t Byte1,
			uint8_t Byte2,
			uint32_t Addr)
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_AAIWPMA(uint8_t Byte1,uint8_t Byte2,uint32_t Addr)
{
	FLASH_CSL();
	SPI_TransByte(FLASH_AAIWProgram);				
	SPI_TransByte((uint8_t)(Addr >> 16));		   //������Ҫд���ݵ���ʼ��ַ
	SPI_TransByte((uint8_t)(Addr >> 8));
	SPI_TransByte((uint8_t)(Addr));       
	SPI_TransByte(Byte1);						   //�����������������
	SPI_TransByte(Byte2);
	FLASH_CSH();
	FLASH_IsFree();
}
#endif
/**************************************************************
�������ƣ� void	FLASH_AAIWPMB(uint8_t Byte1,uint8_t Byte2)
�������ܣ� WORD���ģʽB
��ڲ����� (uint8_t Byte1,
			uint8_t Byte2,
		    )
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
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
�������ƣ� void	FLASH_WriteBytes(uint8_t *pBuff,uint32_t Addr,uint16_t Bytes)
�������ܣ� ���ֽ�д
��ڲ����� (uint8_t *pBuff,
			uint32_t Addr,
			uint16_t Bytes)
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
#if( USE_ALLFUNCTION  )
void	FLASH_WriteBytes(uint8_t *pBuff,uint32_t Addr,uint16_t Bytes)
{
		uint16_t i,temp;
		uint32_t secpos;
		uint16_t secoff;
		uint16_t secremain;     
		//���´���Ϊ������д����Ĵ���
		secpos		=	Addr	/	4096;      		  //������4K����ַ0~511 for     SST25VF016
		secoff		=	Addr	%	4096;             //�������ڵ�ƫ��
		secremain	=	4096	-	secoff;       	  //����ʣ��ռ��С
		if(Bytes <secremain)   temp=1;                //ʣ��ռ������������
		else                                          //ʣ��ռ�С����������
		{
		  i= Bytes - secremain;                       //�жϻ�ռ�˼�������
		  if(i%4096==0)	temp=i/4096+1;
		  else			temp=i/4096+2;
		}
		for(i=0;i<temp;i++)
		{
		  FLASH_OpSectorErase((secpos+i)*4096);       //������Ҫд�����ݵ�����   
		}
  //���´���Ϊ������д��ָ����ַ�Ĵ���
		temp=Bytes >> 1;
		FLASH_OpWREN();
		FLASH_AAIWPMA(pBuff[0], pBuff[1],Addr );     //��ʼд����
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
�������ƣ� void	FLASH_WriteByte(uint8_t Byte,uint32_t Addr,uint8_t EraseEn)
�������ܣ� ���ֽ�д
��ڲ����� (uint8_t *pBuff,
			uint32_t Addr,
			uint16_t Bytes)
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	FLASH_WriteByte(uint8_t Byte,uint32_t Addr,uint8_t EraseEn)
{                            
	if( EraseEn == 0XAA)
	{
		FLASH_OpSectorErase( Addr/4096 );  //�����������	//������ַ 0~511 for w25x16  4096=4k	
	}	
	FLASH_OpWREN();                              
	FLASH_CSL();
	SPI_TransByte(FLASH_ByteProgram );      //����дҳ����		  
	SPI_TransByte((uint8_t)((Addr)>>16));  	//����24bit��ַ 
	SPI_TransByte((uint8_t)((Addr)>>8));  
	SPI_TransByte((uint8_t)Addr);                 
	SPI_TransByte( Byte );					//���ʹ�д������
	FLASH_CSH();
	FLASH_OpWRDI();
	FLASH_IsFree();    
}
/**************************************************************
�������ƣ� void	FLASH_WriteByte(uint8_t Byte,uint32_t Addr,uint8_t EraseEn)
�������ܣ� ���ֽ�д
��ڲ����� (uint8_t *pBuff,
			uint32_t Addr,
			uint16_t Bytes)
���ز����� 
��д���ڣ� 2013.9.5
�������ߣ� ������	 ZeroOne :631640356
˵����
BIT0	BIT1	BIT2	BIT3	BIT4	BIT5	BIT6	BIT7
BUSY	WEL		 BP0	BP1		BP2		BP3		AAI		BPL	
**************************************************************/
void	FLASH_Test(void)
{
	 uint32_t	Temp ,ErrorCnt=0;
	 uint8_t	ReadValue ,i;
	 uint8_t	Table[]="6.�洢��У����,�����ĵȴ�...\r\n";
	 print_string("-------------SPI�洢��������-----------\r\n");
	 Myprintf("1.�洢����ID����: ", FLASH_ReadID());			   
	 Myprintf("2.�洢��������ID����: ", FLASH_ReadJEDEC_ID());		//����������������
	 print_string("3.�洢��������ʼ...\r\n");
	 FLASH_OpChipErase();
	 print_string("4.�洢����������...\r\n");
	 print_string("5.�洢������У�鿪ʼ...\r\n");
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
	 Myprintf("7.�洢��У�����,���ֽ���: ", ErrorCnt);
	 if(ErrorCnt) 	  print_string("8.�ô洢�����ϸ񣬲���ʹ��...\r\n");
	 else			  print_string("9.�ô洢���ϸ񣬿���Ͷ��ʹ��...\r\n");
	 print_string("-------------SPI�洢�����Խ���-----------\r\n");
}													   


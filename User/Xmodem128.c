#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include 	"uart.h"
#include	"Xmodem128.h"
extern		volatile 	uint8_t  UARTBuffer[];
extern		void 		delaySysTick(uint32_t tick);
#define		XMODE_Sleep1S()		delaySysTick(10000)			//���ڵȴ������Ļظ�
#define		XMODE_Sleep15mS()	delaySysTick(120)	//300	//131���ַ���������115200������Ҫ��������15mS
#define		XMODE_Sleep200uS()	delaySysTick(5)

//http://blog.163.com/du_minchao@126/blog/static/107495394201075114028606/
//http://blog.sina.com.cn/s/blog_5d2412000100cp3y.html
uint16_t XOMDE_CRC16( uint8_t *ptr, uint16_t count)
{
	uint16_t crc, i;
	crc = 0;
	while(count--)
	{
		crc = crc ^ (int) *ptr++ << 8;	//��packet_data��ȡһ���ֽ����ݣ�ǿתΪ16Ϊint���ٰѵͰ�λ�Ƶ��߰�λ����ֵ��crc	
		for(i = 0; i < 8; i++)
		{
			if(crc & 0x8000)//�ж����ݵ����λ�����Ƿ�Ϊ1
				crc = crc << 1 ^ 0x1021; //	CRC-ITU
			else			
				crc = crc << 1;
		}
	}
//	return (crc & 0xFFFF);
return 0;
}


uint8_t	XMODE_SendCmdRecPick(uint8_t	Cmd)
{
		 XMODEM128	*Check;
		 uint8_t	Temp ,Retry=0xff;
		 for(Temp=0;Temp<sizeof(XMODEM128);Temp++)	  			//�����ʱ����������
		 {
		 	 UARTBuffer[Temp]	=0x00;
		 }
		 do
		 {
		 	 Check->RecIdle		=0x00;							
			 XMODE_Sleep200uS();
		 }while( Check->RecIdle );		 					    //�ȴ����տ��У������Ǵ�������ݵ��µ�
		 Check	=(XMODEM128 *)	UARTBuffer;
		 Check->TimeOut		=0x01;								//������ʱ����
		 Check->Count		=0x00;								//���½�������
		 Check->RecEn		=0x01;							    //�����������
		 while((LPC_USART->LSR & 0x60) != 0x60);				//�ȴ���������Ϊ��				
    	 LPC_USART->THR =	Cmd;								//��������
		 if(Cmd == 'C')		XMODE_Sleep1S();
		 else				
		 {
		 	  do
			 {
			 	 Check->RecIdle	=0x00;						
				 XMODE_Sleep200uS();
			 }while((!Check->RecIdle ) &&(Retry--));		 	//�ȴ���ʼ��������
			 XMODE_Sleep15mS();
		 }			
		 return (! Check->TimeOut);							    //�������յ�����		  
}
//Order=0  ���������ֿ�
//ORDER=1  ����Ӣ���ֿ�
uint8_t	XMODE_ReceiveData(uint8_t Order)
{
		uint8_t		PacketNumA,PacketNumB,Step;
		uint8_t		CMD , StartOk ,EndCnt,CancelCnt,Retry=0xff;
		uint32_t	FlashAddr;
		XMODEM128	*MyData;
		if(Order)	FlashAddr	=ENZK_Addr;
		else		FlashAddr	=CNZK_Addr;	 
		
		MyData		=(XMODEM128 *)UARTBuffer;
		CMD			='C';										//����XMODEM���ݴ���
		PacketNumA	=0x01;
		EndCnt		=0x00;
		CancelCnt	=0x00;
		StartOk		=FALSE;
	do
	{
		PacketNumB	=~PacketNumA;		
		if ( XMODE_SendCmdRecPick( CMD ) )	 					//��ȷ���յ��˵�һ��/һ������
		{	   
		 	if(( MyData->PacketNumber1 == PacketNumA)&&(MyData->PacketNumber0 == PacketNumB)&&(MyData ->RecEn == 0xAA))
			{
				//��������У��
				StartOk	  =TRUE;
				Retry	  =0xff;	  
				EndCnt	  =0x00;
				CancelCnt =0x00;
				if( XOMDE_CRC16((uint8_t *)(UARTBuffer+3), 130) == 0x00)
				{
					for(Step=3;Step<131;Step++)
					{
					
						if( FlashAddr > FLASH_ROOM)
						{
							XMODE_SendCmdRecPick( DLE );					//��������
							Retry=0x00;										//��ַԽ�磬ֱ��ʧ���˳�
						}		 
					}
					PacketNumA+=1;
					CMD		=ACK;
				}
				else	CMD =NAK;
			}
			else	CMD		=NAK;
		}
		else if( StartOk !=  TRUE)
		{
			   CMD		='C';
			   PacketNumA=0x01;
			   Retry	--;
		}					 
		else
		{
		 		CMD		=NAK;
				Retry	--;	
		}
		if( MyData->StartOfHeader == EOT)		  				
		{
			   EndCnt	++;
			   CMD		=NAK;	   //�Ա��ڿ����ٴεõ�EOT�ظ�
		}
		if( MyData->StartOfHeader == CAN)		  				
		{
			   CancelCnt ++;
			   CMD		=NAK;	   //�Ա��ڿ����ٴεõ�CAN�ظ�
		}
												
	}while((EndCnt != 5)&&(CancelCnt != 5)&& Retry );	//����Ҫ���յ����������־����Ϊ�ǽ���
	XMODE_SendCmdRecPick( ACK );
	if(EndCnt == 5)		return 1;
	else				return 0;
}





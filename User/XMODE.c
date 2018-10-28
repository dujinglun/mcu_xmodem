#include 	"lpc11Uxx.h"
#include	"XMODE.h"
#include  <stdint.h>

uint8_t		Buff[135];
uint16_t	XMODE_CheckSum(uint8_t *pBuff,uint8_t Size)
{
	uint16_t CheckSum, i;
	CheckSum = 0;
	while(Size--)
	{
		CheckSum = CheckSum ^ (int) *pBuff++ << 8;			
		for(i = 0; i < 8; i++)								
		{
			if(CheckSum & 0x8000)	CheckSum = CheckSum << 1 ^ 0x1021; 
			else					CheckSum = CheckSum << 1;
		}
	}
	return CheckSum;	
}

void		XMODE_Init(uint32_t	UART_Bps)
{
	uint16_t	ByteUseTime;
	ByteUseTime		=1000000 / UART_Bps;		//1000 000 uS / 115 200		
}
uint8_t		XMODE_GetAData(uint8_t *pData)
{
		SetTick();
		do
		{
			if(XMODE_GetByte(pData))  return 1;
		}while( CheckDelay(300) );		//20mS
		return 		0;
}
uint8_t		XMODE_GetBData(uint8_t *pData)
{
		SetTick();
		do
		{
			if(XMODE_GetByte(pData))  return 1;
		}while( CheckDelay(3) );		//300uS
		return 		0;
}
uint8_t		XMODE_GetPack(uint8_t	Cmd)
{
		 uint8_t	Temp,Retry,Data;
		 for(Temp=0;Temp<135;Temp++)	Buff[Temp]	=0x00;	 	 
		 Retry	= 100;
		 Temp	= 0;
		 while(Retry--)
		 {
			 if(Cmd == 'C')		
			 {
	 			SetTick();
	 			while( CheckDelay(5000));		//休眠0.5S 
	 		 }
			 XMODE_SendByte(Cmd);
		 	if( XMODE_GetAData( &Data ))
			 {					 		
		 		do
		 		{
		 			if(Temp >= 133)	return Temp;		
		 			Buff[Temp++]	=Data;	
		 		}while( XMODE_GetBData( &Data) );
		 		return  (Temp);
			 } 			
 		 }
		  return  0;							
}


uint8_t		XMODE_Loading(void)
{
static		uint8_t		NumA,NumB,CMD,StartOk,Retry=0xff;
		XMODEM_TYPE	*MyData;		
		MyData		=(XMODEM_TYPE *)Buff;
		CMD			='C';										//启动XMODEM数据传输
		NumA		=0x01;
		StartOk		=FALSE;
	do
	{
		NumB	=~NumA;		
		if ( XMODE_GetPack( CMD ) == 133)	 					//正确的收到了第一包/一个数据
		{	   
		 	if(( MyData->PackNum1 == NumA)&&(MyData->PackNum0 == NumB))
			{
				//加入数据校验
				StartOk	  =TRUE;
				Retry	  =200;
				if( XMODE_CheckSum(Buff + 3, 130) == 0x00)
				{
					 NumA++;
					 CMD =XMODE_CMD_ACK;
				}
				else CMD =XMODE_CMD_NAK;
			}else	 CMD =XMODE_CMD_NAK;
		}
		else if( StartOk !=  TRUE)
		{
			   CMD	  ='C';
			   NumA   =0x01;
			   Retry  --;
		}					 
		else
		{
		 		CMD		=XMODE_CMD_NAK;
				Retry	--;	
		}											
	}while(( MyData->Header!= XMODE_CMD_EOT)&&(MyData->Header != XMODE_CMD_CAN)&& Retry );
	XMODE_SendByte( XMODE_CMD_ACK );
	return		(MyData->Header == XMODE_CMD_EOT);	
}


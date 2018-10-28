#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include 	"uart.h"
#include	"Xmodem128.h"
extern		volatile 	uint8_t  UARTBuffer[];
extern		void 		delaySysTick(uint32_t tick);
#define		XMODE_Sleep1S()		delaySysTick(10000)			//用于等待请求后的回复
#define		XMODE_Sleep15mS()	delaySysTick(120)	//300	//131个字符，波特率115200传输需要花销顶多15mS
#define		XMODE_Sleep200uS()	delaySysTick(5)

//http://blog.163.com/du_minchao@126/blog/static/107495394201075114028606/
//http://blog.sina.com.cn/s/blog_5d2412000100cp3y.html
uint16_t XOMDE_CRC16( uint8_t *ptr, uint16_t count)
{
	uint16_t crc, i;
	crc = 0;
	while(count--)
	{
		crc = crc ^ (int) *ptr++ << 8;	//从packet_data中取一个字节数据，强转为16为int，再把低八位移到高八位，赋值给crc	
		for(i = 0; i < 8; i++)
		{
			if(crc & 0x8000)//判断数据的最高位数据是否为1
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
		 for(Temp=0;Temp<sizeof(XMODEM128);Temp++)	  			//清除过时的垃圾数据
		 {
		 	 UARTBuffer[Temp]	=0x00;
		 }
		 do
		 {
		 	 Check->RecIdle		=0x00;							
			 XMODE_Sleep200uS();
		 }while( Check->RecIdle );		 					    //等待接收空闲，可能是错误的数据导致的
		 Check	=(XMODEM128 *)	UARTBuffer;
		 Check->TimeOut		=0x01;								//开启超时计数
		 Check->Count		=0x00;								//重新接收数据
		 Check->RecEn		=0x01;							    //允许接收数据
		 while((LPC_USART->LSR & 0x60) != 0x60);				//等待发送数据为空				
    	 LPC_USART->THR =	Cmd;								//发送数据
		 if(Cmd == 'C')		XMODE_Sleep1S();
		 else				
		 {
		 	  do
			 {
			 	 Check->RecIdle	=0x00;						
				 XMODE_Sleep200uS();
			 }while((!Check->RecIdle ) &&(Retry--));		 	//等待开始接收数据
			 XMODE_Sleep15mS();
		 }			
		 return (! Check->TimeOut);							    //正常接收到数据		  
}
//Order=0  下载中文字库
//ORDER=1  下载英文字库
uint8_t	XMODE_ReceiveData(uint8_t Order)
{
		uint8_t		PacketNumA,PacketNumB,Step;
		uint8_t		CMD , StartOk ,EndCnt,CancelCnt,Retry=0xff;
		uint32_t	FlashAddr;
		XMODEM128	*MyData;
		if(Order)	FlashAddr	=ENZK_Addr;
		else		FlashAddr	=CNZK_Addr;	 
		
		MyData		=(XMODEM128 *)UARTBuffer;
		CMD			='C';										//启动XMODEM数据传输
		PacketNumA	=0x01;
		EndCnt		=0x00;
		CancelCnt	=0x00;
		StartOk		=FALSE;
	do
	{
		PacketNumB	=~PacketNumA;		
		if ( XMODE_SendCmdRecPick( CMD ) )	 					//正确的收到了第一包/一个数据
		{	   
		 	if(( MyData->PacketNumber1 == PacketNumA)&&(MyData->PacketNumber0 == PacketNumB)&&(MyData ->RecEn == 0xAA))
			{
				//加入数据校验
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
							XMODE_SendCmdRecPick( DLE );					//撤销传输
							Retry=0x00;										//地址越界，直接失败退出
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
			   CMD		=NAK;	   //以便于可以再次得到EOT回复
		}
		if( MyData->StartOfHeader == CAN)		  				
		{
			   CancelCnt ++;
			   CMD		=NAK;	   //以便于可以再次得到CAN回复
		}
												
	}while((EndCnt != 5)&&(CancelCnt != 5)&& Retry );	//必须要接收到多个结束标志才认为是结束
	XMODE_SendCmdRecPick( ACK );
	if(EndCnt == 5)		return 1;
	else				return 0;
}





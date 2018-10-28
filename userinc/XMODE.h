#ifndef		__XMODE_H__
#define		__XMODE_H__

#include <stdint.h>

extern		uint8_t 	UART_GetChar( uint8_t *Byte );
extern		void 		UART_SendChar( uint8_t Byte );

extern		void		SetTick(void);
extern		uint8_t 	CheckDelay (uint32_t Time);		//100uS
#define		XMODE_SendByte		UART_SendChar
#define		XMODE_GetByte		UART_GetChar
#define		FALSE				0
#define		TRUE				1

typedef		enum
{
	XMODE_CMD_SOH	=0x01,					//数据块开始
	XMODE_CMD_EOT	=0X04,					//数据发送结束
	XMODE_CMD_ACK	=0X06,					//认可响应
	XMODE_CMD_NAK	=0X15,					//'C'		 ///0x15	   //不认可响应
	XMODE_CMD_DEL	=0X10,					//中止数据传输
	XMODE_CMD_XON	=0X11,					//继续传输
	XMODE_CMD_XOFF	=0X19,					//数据传输暂停
	XMODE_CMD_SYN	=0X16,					//同步
	XMODE_CMD_CAN	=0X18					//撤销传输
}XMODE_CMD;

typedef		struct	
{
	  uint8_t		Header;		   			//01hex
	  uint8_t		PackNum1;		   		//包序号，从01开始，发送一个包加1 加到FF然后从01开始循环
	  uint8_t		PackNum0;		   		//包序号 反码
	  uint8_t		PackData[128];	    	//对于不满128字节的数据包，则用0X1A来填充
	  uint16_t		CRC16;			    	//只对数据包进行校验
}XMODEM_TYPE;

extern		uint8_t		XMODE_Loading(void);


#endif

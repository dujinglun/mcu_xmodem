#ifndef __XMODE_H__
#define __XMODE_H__
typedef		struct	
{
	  uint8_t		StartOfHeader;		   //01hex
	  uint8_t		PacketNumber1;		   //包序号，从01开始，发送一个包加1 加到FF然后从01开始循环
	  uint8_t		PacketNumber0;		   //包序号 反码
	  uint8_t		PacketData[128];	   //对于不满128字节的数据包，则用0X1A来填充
	  uint16_t		CheckSum;			   //只对数据包进行校验
	  uint16_t		RecIdle;			   //接收数据空闲
	  uint8_t		TimeOut;			   //超时标志
	  uint8_t		RecEn;				   //允许接收数据
	  uint8_t		Count;				   //接收数据计数
}XMODEM128;
#define			SOH			0x01	   //数据块开始
#define			EOT			0x04	   //数据发送结束
#define			ACK			0x06	   //认可响应
#define			NAK			0x15	   //'C'		 ///0x15	   //不认可响应
#define			DLE			0x10	   //中止数据传输
#define			XON			0X11	   //继续传输
#define			XOFF		0x19	   //数据传输暂停
#define			SYN			0x16	   //同步
#define		    CAN			0x18	   //撤销传输
uint8_t	XMODE_ReceiveData(uint8_t Order);
/************************************************************************
*************************************************************************/
#endif


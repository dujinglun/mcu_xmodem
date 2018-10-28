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
	XMODE_CMD_SOH	=0x01,					//���ݿ鿪ʼ
	XMODE_CMD_EOT	=0X04,					//���ݷ��ͽ���
	XMODE_CMD_ACK	=0X06,					//�Ͽ���Ӧ
	XMODE_CMD_NAK	=0X15,					//'C'		 ///0x15	   //���Ͽ���Ӧ
	XMODE_CMD_DEL	=0X10,					//��ֹ���ݴ���
	XMODE_CMD_XON	=0X11,					//��������
	XMODE_CMD_XOFF	=0X19,					//���ݴ�����ͣ
	XMODE_CMD_SYN	=0X16,					//ͬ��
	XMODE_CMD_CAN	=0X18					//��������
}XMODE_CMD;

typedef		struct	
{
	  uint8_t		Header;		   			//01hex
	  uint8_t		PackNum1;		   		//����ţ���01��ʼ������һ������1 �ӵ�FFȻ���01��ʼѭ��
	  uint8_t		PackNum0;		   		//����� ����
	  uint8_t		PackData[128];	    	//���ڲ���128�ֽڵ����ݰ�������0X1A�����
	  uint16_t		CRC16;			    	//ֻ�����ݰ�����У��
}XMODEM_TYPE;

extern		uint8_t		XMODE_Loading(void);


#endif

#ifndef __XMODE_H__
#define __XMODE_H__
typedef		struct	
{
	  uint8_t		StartOfHeader;		   //01hex
	  uint8_t		PacketNumber1;		   //����ţ���01��ʼ������һ������1 �ӵ�FFȻ���01��ʼѭ��
	  uint8_t		PacketNumber0;		   //����� ����
	  uint8_t		PacketData[128];	   //���ڲ���128�ֽڵ����ݰ�������0X1A�����
	  uint16_t		CheckSum;			   //ֻ�����ݰ�����У��
	  uint16_t		RecIdle;			   //�������ݿ���
	  uint8_t		TimeOut;			   //��ʱ��־
	  uint8_t		RecEn;				   //�����������
	  uint8_t		Count;				   //�������ݼ���
}XMODEM128;
#define			SOH			0x01	   //���ݿ鿪ʼ
#define			EOT			0x04	   //���ݷ��ͽ���
#define			ACK			0x06	   //�Ͽ���Ӧ
#define			NAK			0x15	   //'C'		 ///0x15	   //���Ͽ���Ӧ
#define			DLE			0x10	   //��ֹ���ݴ���
#define			XON			0X11	   //��������
#define			XOFF		0x19	   //���ݴ�����ͣ
#define			SYN			0x16	   //ͬ��
#define		    CAN			0x18	   //��������
uint8_t	XMODE_ReceiveData(uint8_t Order);
/************************************************************************
*************************************************************************/
#endif


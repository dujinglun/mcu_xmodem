#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include	"Key.h"
/**************************************************************
�������ƣ� void	Key_Delay(void)
�������ܣ� ����ʱ
��ڲ����� 
���ز����� 
��д���ڣ� 2013.8.30
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	Key_Delay(void)
{
	uint8_t	Dly1,Dly2,Dly3;
	for(Dly3=0;Dly3<48;Dly3++)
	for(Dly1=0;Dly1<100;Dly1++)
	for(Dly2=0;Dly2<110;Dly2++);
}
/**************************************************************
�������ƣ� void	Key_Init(void)
�������ܣ� ��ʼ������IO
��ڲ����� 
���ز����� 
��д���ڣ� 2013.8.30
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
void	Key_Init(void)
{
	LPC_IOCON->TDO_PIO0_13	|=0x01;	
	GPIOSetDir( KEY_PORT, 	KEY_UP_BIT, 	0 );
	GPIOSetDir( KEY_PORT, 	KEY_DN_BIT, 	0 );
	GPIOSetDir( KEY_PORT, 	KEY_OK_BIT, 	0 );
	GPIOSetDir( KEY_PORT, 	KEY_BK_BIT, 	0 );			
}
/**************************************************************
�������ƣ� uchar	Key_GetValue(void)
�������ܣ� ���һ������ֵ
��ڲ����� 
���ز����� 
��д���ڣ� 2013.8.30
�������ߣ� ������	 ZeroOne :631640356
˵����	
**************************************************************/
uint8_t	Key_GetValue(void)
{
		uint32_t	PinValue ,Temp;
		uint8_t		KeyValue;
		PinValue	=LPC_GPIO->PIN[ KEY_PORT ] & KEY_CHECK;
		if( PinValue != KEY_CHECK)
		{
			 Key_Delay();
			 PinValue	=LPC_GPIO->PIN[ KEY_PORT ] & KEY_CHECK;
			 do
			 {
			 	   Temp	=LPC_GPIO->PIN[ KEY_PORT ] & KEY_CHECK;
			 }while(Temp != KEY_CHECK);
			 switch( PinValue ^ KEY_CHECK)
			 {
			 	case (1<<KEY_UP_BIT):	KeyValue	=KEY_UP;	break;
				case (1<<KEY_DN_BIT):	KeyValue	=KEY_DN;	break;
				case (1<<KEY_OK_BIT):	KeyValue	=KEY_OK;	break;
				case (1<<KEY_BK_BIT):	KeyValue	=KEY_BK;	break;
				default	:			KeyValue	=KEY_NULL;	break;
			 }
		}
		else
		{
				KeyValue	=KEY_NULL;
		}
		return  KeyValue;
}


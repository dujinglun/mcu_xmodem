#include 	"lpc11Uxx.h"                        /* LPC11xx definitions */
#include 	"type.h"
#include 	"gpio.h"
#include 	"uart.h"
#include	"Xmodem128.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DELAY		5000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t TimeTick = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* SysTick interrupt happens every 100uS*/ 
void SysTick_Handler(void)
{
	TimeTick++;
}
void		SetTick(void)
{
	SysTick->CTRL	&= ~SysTick_CTRL_ENABLE_Msk;
	TimeTick		 =0;
	SysTick->VAL   	 =0;
	SysTick->CTRL	|=  SysTick_CTRL_ENABLE_Msk;
}
uint8_t 	CheckDelay (uint32_t Time)
{
	return (TimeTick < Time);
}
void	main(void)
{	
	SystemInit();
	UARTInit(115200);	
	SysTick_Config( SYSTICK_DELAY );
	GPIOSetDir( 0, 7, 1 );
	for(;;)
	{
		XMODE_Loading();
#if 0
		GPIOSetBitValue(0,7,0);
		GPIOSetBitValue(0,7,1);	
#endif
	}
}


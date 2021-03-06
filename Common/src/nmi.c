/****************************************************************************
 *   $Id:: nmi.c 6172 2011-01-13 18:22:51Z usb00423                         $
 *   Project: NXP LPC122x NMI interrupt example
 *
 *   Description:
 *     This file contains NMI interrupt handler code example.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include "LPC11Uxx.h"
#include "nmi.h"

#if NMI_ENABLED
volatile uint32_t NMI_Counter[MAX_NMI_NUM];

/*****************************************************************************
** Function name:		NMI_Handler
**
** Descriptions:		NMI interrupt handler
** parameters:		None			 
** 						
** Returned value:	None
** 
*****************************************************************************/
void NMI_Handler( void )
{
  uint32_t regVal;

  regVal = LPC_SYSCON->INT_NMI_SRC_CTRL;
  regVal &=	~0x80000000;
  if ( regVal < MAX_NMI_NUM )
  {
    if ( regVal == TIMER_16_0_IRQn )
	{
	  /* Use TIMER16_0_IRQHandler as example for real application. */ 	
	  LPC_TMR16B0->IR = 0xFF;	/* Clear timer16_0 interrupt */
	}
	else if ( regVal == TIMER_16_1_IRQn )
	{
	  /* Use TIMER16_1_IRQHandler as example for real application. */	
	  LPC_TMR16B1->IR = 0xFF;	/* Clear timer16_1 interrupt */
	}
    else if ( regVal == TIMER_32_0_IRQn )
	{
	  /* Use TIMER32_0_IRQHandler as example for real application. */ 	
	  LPC_TMR32B0->IR = 0xFF;	/* Clear timer32_0 interrupt */
	}
	else if ( regVal == TIMER_32_1_IRQn )
	{
	  /* Use TIMER32_0_IRQHandler as example for real application. */ 	
	  LPC_TMR32B1->IR = 0xFF;	/* Clear timer32_1 interrupt */
	}
	NMI_Counter[regVal]++; 
  }
  return;
}

/*****************************************************************************
** Function name:		NMI_Init
**
** Descriptions:		NMI initialization
** parameters:			NMI number			 
** 						
** Returned value:		None
** 
*****************************************************************************/
void NMI_Init( uint32_t NMI_num )
{
  uint32_t i;

  for ( i = 0; i < MAX_NMI_NUM; i++ )
  {
    NMI_Counter[i] = 0x0;
  }
  LPC_SYSCON->INT_NMI_SRC_CTRL = 0x80000000|NMI_num;
  return;
}

#endif

/****************************************************************************
 *   $Id:: uart.c 6957 2011-03-23 23:03:48Z usb00423                        $
 *   Project: NXP LPC11xx UART example
 *
 *   Description:
 *     This file contains UART code example which include UART 
 *     initialization, UART interrupt handler, and related APIs for 
 *     UART access.
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
#include 	"LPC11Uxx.h"
#include 	"uart.h"
#include	"Xmodem128.h"
#define		USE_XMODEM		1
#if( USE_XMODEM )
	#define 	BUFSIZE        (sizeof(XMODEM128))
#else
	#define 	BUFSIZE         0x20
#endif
volatile 	uint8_t  		UARTBuffer[BUFSIZE];
volatile 	uint32_t 		UARTStatus;
volatile 	uint8_t  		UARTTxEmpty = 1;
volatile 	uint32_t 		UARTCount = 0;

/*****************************************************************************
** Function name:		UART_IRQHandler
**
** Descriptions:		UART interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART_IRQHandler(void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
#if(USE_XMODEM)
	XMODEM128	*XmodemRec;
	XmodemRec	=(XMODEM128 *)UARTBuffer;	
#endif
  IIRValue = LPC_USART->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if (IIRValue == IIR_RLS)  /* Receive Line Status */
  {
    LSRValue = LPC_USART->LSR;
    /* Receive Line Status */
    if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
    {
      /* There are errors or break interrupt */
      /* Read LSR will clear the interrupt */
      UARTStatus = LSRValue;
      Dummy = LPC_USART->RBR;	/* Dummy read on RX to clear 
								interrupt, then bail out */
      return;
    }
    if (LSRValue & LSR_RDR)	/* Receive Data Ready */			
    {
      /* If no error on RLS, normal ready, save into the data buffer. */
      /* Note: read RBR will clear the interrupt */
#if(USE_XMODEM)
      XmodemRec->RecIdle  ++;
	  if(XmodemRec->RecEn ==0x01)
	  {
	  	   XmodemRec->TimeOut	=0x00;
		  UARTBuffer[ XmodemRec->Count++ ]	=LPC_USART->RBR;
		  if( XmodemRec->Count >= 133)			 //XOMDE结构体中最后2为是控制用的，不能存数据
		  {
		  	   XmodemRec->Count	=0x00;
			   XmodemRec->RecEn =0xAA;					//接收完成一个数据包后，要等待处理，才能继续接收
		  }
	  }
	  else
	  {
	  	  Dummy =LPC_USART->RBR;
	  }
#else
      UARTBuffer[UARTCount++] = LPC_USART->RBR;
      if (UARTCount == BUFSIZE)
      {
        UARTCount = 0;		/* buffer overflow */
      }	
#endif
    }
  }
  else if (IIRValue == IIR_RDA)	/* Receive Data Available */
  {
    /* Receive Data Available */
#if(USE_XMODEM)
	  XmodemRec->RecIdle  ++;
	  if(XmodemRec->RecEn == 0x01)
	  {
	  	   XmodemRec->TimeOut	=0x00;
		  UARTBuffer[ XmodemRec->Count++ ]	=LPC_USART->RBR;
		  if( XmodemRec->Count >= 133)			 //XOMDE结构体中最后2为是控制用的，不能存数据
		  {
		  	   XmodemRec->Count	=0x00;
			   XmodemRec->RecEn =0xAA;					//接收完成一个数据包后，要等待处理，才能继续接收
		  }
	  }
	  else
	  {
	  	  Dummy =LPC_USART->RBR;
	  }
#else
      UARTBuffer[UARTCount++] = LPC_USART->RBR;
      if (UARTCount == BUFSIZE)
      {
        UARTCount = 0;		/* buffer overflow */
      }	
#endif
  }
  else if (IIRValue == IIR_CTI)	/* Character timeout indicator */
  {
    /* Character Time-out indicator */
    UARTStatus |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if (IIRValue == IIR_THRE)	/* THRE, transmit holding register empty */
  {
    /* THRE interrupt */
    LSRValue = LPC_USART->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
    if (LSRValue & LSR_THRE)
    {
      UARTTxEmpty = 1;
    }
    else
    {
      UARTTxEmpty = 0;
    }
  }
  return;
}

#if MODEM_TEST
/*****************************************************************************
** Function name:		ModemInit
**
** Descriptions:		Initialize UART0 port as modem, setup pin select.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void ModemInit( void )
{
  
  LPC_IOCON->PIO0_7 &= ~0x07;     /* UART I/O config */
  LPC_IOCON->PIO0_7 |= 0x01;      /* UART CTS */
  LPC_IOCON->PIO0_17 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO0_17 |= 0x01;     /* UART RTS */
#if 1
  LPC_IOCON->PIO1_13 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_13 |= 0x01;     /* UART DTR */ 
  LPC_IOCON->PIO1_14 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_14 |= 0x01;     /* UART DSR */
  LPC_IOCON->PIO1_15 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_15 |= 0x01;     /* UART DCD */
  LPC_IOCON->PIO1_16 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_16 |= 0x01;     /* UART RI */

#else
  LPC_IOCON->PIO1_19 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_19 |= 0x01;     /* UART DTR */
  LPC_IOCON->PIO1_20 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_20 |= 0x01;     /* UART DSR */
  LPC_IOCON->PIO1_21 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_21 |= 0x01;     /* UART DCD */
  LPC_IOCON->PIO1_22 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_22 |= 0x01;     /* UART RI */
#endif
  LPC_USART->MCR = 0xC0;          /* Enable Auto RTS and Auto CTS. */			
  return;
}
#endif

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART0 port, setup pin select,
**				clock, parity, stop bits, FIFO, etc.
**
** parameters:			UART baudrate
** Returned value:		None
** 
*****************************************************************************/
void UARTInit(uint32_t baudrate)
{
  uint32_t Fdiv;
  uint32_t regVal;

  UARTTxEmpty = 1;
  UARTCount = 0;
  
  NVIC_DisableIRQ(UART_IRQn);
  
  /* Select only one location from below. */
#if 1
  LPC_IOCON->PIO0_18 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO0_18 |= 0x01;     /* UART RXD */
  LPC_IOCON->PIO0_19 &= ~0x07;	
  LPC_IOCON->PIO0_19 |= 0x01;     /* UART TXD */
#endif
#if 0
  LPC_IOCON->PIO1_14 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO1_14 |= 0x03;     /* UART RXD */
  LPC_IOCON->PIO1_13 &= ~0x07;	
  LPC_IOCON->PIO1_13 |= 0x03;     /* UART TXD */
#endif
#if 0
  LPC_IOCON->PIO1_26 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO1_26 |= 0x02;     /* UART RXD */
  LPC_IOCON->PIO1_27 &= ~0x07;	
  LPC_IOCON->PIO1_27 |= 0x02;     /* UART TXD */
#endif

  /* Enable UART clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
  LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

  LPC_USART->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
  regVal = LPC_SYSCON->UARTCLKDIV;
  Fdiv = ((SystemCoreClock/regVal)/16)/baudrate ;	/*baud rate */

  LPC_USART->DLM = Fdiv / 256;							
  LPC_USART->DLL = Fdiv % 256;
  LPC_USART->LCR = 0x03;		/* DLAB = 0 */
  LPC_USART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

  /* Read to clear the line status. */
  regVal = LPC_USART->LSR;

  /* Ensure a clean start, no data in either TX or RX FIFO. */
  while (( LPC_USART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
  while ( LPC_USART->LSR & LSR_RDR )
  {
	regVal = LPC_USART->RBR;	/* Dump data from RX FIFO */
  }
 
  /* Enable the UART Interrupt */
//  NVIC_EnableIRQ(UART_IRQn);

#if 0
  LPC_USART->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART interrupt */
#else
  LPC_USART->IER = IER_RBR | IER_RLS;	/* Enable UART interrupt */
#endif
  return;
}
/*****************************************************************************
** Function name:		print_string
**
** Descriptions:		print out string on the terminal
**
** parameters:			pointer to the string end with NULL char.
** Returned value:		none.
** 
*****************************************************************************/
void	UART_SendChar(uint8_t Byte)
{
    while((LPC_USART->LSR & 0x60) != 0x60);
    LPC_USART->THR = Byte;
}

/*****************************************************************************
** Function name:		get_key
**
** Descriptions:		Get a character from the terminal
**
** parameters:			None
** Returned value:		character, zero is none.
** 
*****************************************************************************/
uint8_t UART_GetChar( uint8_t *Byte )
{
	  if(LPC_USART->LSR & 0x01)
	  {
	  	*Byte = LPC_USART->RBR;
	  	 return 1;
	  }
	  	return 0;
}

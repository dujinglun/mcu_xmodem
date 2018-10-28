#line 1 "..\\User\\SPI_FLASH.c"
#line 1 "..\\Common\\inc\\lpc11Uxx.h"

 














 





 



 










  #pragma anon_unions


  

typedef enum {

  Reset_IRQn                        = -15,   
  NonMaskableInt_IRQn               = -14,   
  HardFault_IRQn                    = -13,   
  SVCall_IRQn                       = -5,    
  DebugMonitor_IRQn                 = -4,    
  PendSV_IRQn                       = -2,    
  SysTick_IRQn                      = -1,    

FLEX_INT0_IRQn                = 0,         
  FLEX_INT1_IRQn                = 1,
  FLEX_INT2_IRQn                = 2,
  FLEX_INT3_IRQn                = 3,
  FLEX_INT4_IRQn                = 4,   
  FLEX_INT5_IRQn                = 5,        
  FLEX_INT6_IRQn                = 6,        
  FLEX_INT7_IRQn                = 7,        
  GINT0_IRQn                    = 8,         
  GINT1_IRQn                    = 9,         
  Reserved0_IRQn                = 10,        
  Reserved1_IRQn                = 11,       
  Reserved2_IRQn                = 12,       
  Reserved3_IRQn                = 13,       
  SSP1_IRQn                     = 14,        
  I2C_IRQn                      = 15,        
  TIMER_16_0_IRQn               = 16,        
  TIMER_16_1_IRQn               = 17,        
  TIMER_32_0_IRQn               = 18,        
  TIMER_32_1_IRQn               = 19,        
  SSP0_IRQn                     = 20,        
  UART_IRQn                     = 21,        
  USB_IRQn                      = 22,        
  USB_FIQn                      = 23,        
  ADC_IRQn                      = 24,        
  WDT_IRQn                      = 25,          
  BOD_IRQn                      = 26,        
  FMC_IRQn                      = 27,        
  Reserved4_IRQn                = 28,        
  Reserved5_IRQn                = 29,        
  USBWakeup_IRQn                = 30,        
  Reserved6_IRQn                = 31,        
} IRQn_Type;




 

   




   

#line 1 "..\\Common\\inc\\core_cm0.h"
 




















 























  







 




 






 

 











#line 93 "..\\Common\\inc\\core_cm0.h"

#line 1 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"
 
 





 









#line 25 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"

     







     










     











#line 260 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"



 


#line 95 "..\\Common\\inc\\core_cm0.h"
#line 1 "..\\Common\\inc\\core_cmInstr.h"
 




















 





 



 


 




 







 







 






 








 







 







 









 









 



static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}









 



static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}



#line 260 "..\\Common\\inc\\core_cmInstr.h"



#line 772 "..\\Common\\inc\\core_cmInstr.h"

   

#line 96 "..\\Common\\inc\\core_cm0.h"
#line 1 "..\\Common\\inc\\core_cmFunc.h"
 




















 




 



 


 

 
 






 



static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}








 



static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}








 



static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}








 



static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}








 



static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}








 



static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}








 



static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}








 



static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}








 



static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}








 



static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}








 



static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}

 

#line 312 "..\\Common\\inc\\core_cmFunc.h"


#line 348 "..\\Common\\inc\\core_cmFunc.h"


#line 840 "..\\Common\\inc\\core_cmFunc.h"

 


#line 97 "..\\Common\\inc\\core_cm0.h"









 
#line 114 "..\\Common\\inc\\core_cm0.h"

 





 







 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                              
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IPR[8];                   
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;                                      
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;                                      
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
       uint32_t RESERVED2[2];                                   
  volatile uint32_t DFSR;                     
} SCB_Type;                                                

 















 



























 















 









 






 



 















 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 

































 






 









 




 
 
 











 





 





 



 



 

 
 











 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}










 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) | 
        (((priority << (8 - 1)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IPR[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IPR[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 1)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}













 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 1)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IPR[( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 1)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR  = ((0x5FA << 16)      | 
                 (1UL << 2));
  __dsb(0xF);                                                                    
  while(1);                                                     
}

 



 



 











 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFUL << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<1) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->CTRL  = (1UL << 2) | 
                   (1UL << 1)   | 
                   (1UL << 0);                     
  return (0);                                                   
}



 












 
#line 100 "..\\Common\\inc\\lpc11Uxx.h"
#line 1 "..\\Common\\inc\\system_LPC11Uxx.h"
 





















 









#line 34 "..\\Common\\inc\\system_LPC11Uxx.h"

extern uint32_t SystemCoreClock;      










 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 101 "..\\Common\\inc\\lpc11Uxx.h"



 









 

typedef struct {                             
  volatile uint32_t CONSET;                      
  volatile const  uint32_t STAT;                        
  volatile uint32_t DAT;                         
  volatile uint32_t ADR0;                        
  volatile uint32_t SCLH;                        
  volatile uint32_t SCLL;                        
  volatile uint32_t CONCLR;                      
  volatile uint32_t MMCTRL;                      
  volatile uint32_t ADR1;                        
  volatile uint32_t ADR2;                        
  volatile uint32_t ADR3;                        
  volatile const  uint32_t DATA_BUFFER;                 
union{
  volatile uint32_t MASK[4];                     
  struct{
  volatile uint32_t MASK0;
  volatile uint32_t MASK1;
  volatile uint32_t MASK2;
  volatile uint32_t MASK3;
  };
  };
} LPC_I2C_Type;









 

typedef struct {                             
  volatile uint32_t MOD;                         
  volatile uint32_t TC;                          
  volatile uint32_t FEED;                        
  volatile const  uint32_t TV;                          
  volatile uint32_t CLKSEL;                      
  volatile uint32_t WARNINT;                     
  volatile uint32_t WINDOW;                      
} LPC_WWDT_Type;









 

typedef struct {                             
  
  union {
    volatile uint32_t DLL;                       
    volatile  uint32_t THR;                       
    volatile const  uint32_t RBR;                       
  };
  
  union {
    volatile uint32_t IER;                       
    volatile uint32_t DLM;                       
  };
  
  union {
    volatile  uint32_t FCR;                       
    volatile const  uint32_t IIR;                       
  };
  volatile uint32_t LCR;                         
  volatile uint32_t MCR;                         
  volatile const  uint32_t LSR;                         
  volatile const  uint32_t MSR;                         
  volatile uint32_t SCR;                         
  volatile uint32_t ACR;                         
  volatile uint32_t ICR;                         
  volatile uint32_t FDR;                         
  volatile uint32_t OSR;                         
  volatile uint32_t TER;                         
  volatile const  uint32_t RESERVED0[3];
  volatile uint32_t HDEN;                        
  volatile const  uint32_t RESERVED1;
  volatile uint32_t SCICTRL;                     
  volatile uint32_t RS485CTRL;                   
  volatile uint32_t RS485ADRMATCH;               
  volatile uint32_t RS485DLY;                    
  volatile uint32_t SYNCCTRL; 
} LPC_USART_Type;









 

typedef struct {                             
  volatile uint32_t IR;                          
  volatile uint32_t TCR;                         
  volatile uint32_t TC;                          
  volatile uint32_t PR;                          
  volatile uint32_t PC;                          
  volatile uint32_t MCR;                         
  union {
  volatile uint32_t MR[4];                       
  struct{
  volatile uint32_t MR0;                         
  volatile uint32_t MR1;                         
  volatile uint32_t MR2;                         
  volatile uint32_t MR3;                         
  };
  };
  volatile uint32_t CCR;                         
  union{
  volatile const  uint32_t CR[4];                       
    struct{
  volatile const  uint32_t CR0;			     
  volatile const  uint32_t CR1;			     
  volatile const  uint32_t CR2;			     
  volatile const  uint32_t CR3;			     
  };
  };
volatile uint32_t EMR;                         
  volatile const  uint32_t RESERVED0[12];
  volatile uint32_t CTCR;                        
  volatile uint32_t PWMC;                        
} LPC_CTxxBx_Type;










 

typedef struct {                             
  volatile uint32_t CR;                          
  volatile uint32_t GDR;                         
  volatile const  uint32_t RESERVED0[1];
  volatile uint32_t INTEN;                       
  union{
  volatile const  uint32_t DR[8];                       
    struct{
  volatile uint32_t DR0;                      	 
  volatile uint32_t DR1;                      	 
  volatile uint32_t DR2;                      	 
  volatile uint32_t DR3;                      	 
  volatile uint32_t DR4;                      	 
  volatile uint32_t DR5;                      	 
  volatile uint32_t DR6;                      	 
  volatile uint32_t DR7;                      	 
  };
  };
  volatile const  uint32_t STAT;                        
} LPC_ADC_Type;









 

typedef struct {                             
  volatile uint32_t PCON;                        
  union{
  volatile uint32_t GPREG[4];                    
  struct{
  volatile uint32_t GPREG0;                   	 
  volatile uint32_t GPREG1;                   	 
  volatile uint32_t GPREG2;                   	 
  volatile uint32_t GPREG3;                   	 
  };
  };
} LPC_PMU_Type;









 

typedef struct {                             
  volatile const  uint32_t RESERVED0[4];
  volatile uint32_t FLASHCFG;                    
  volatile const  uint32_t RESERVED1[3];
  volatile uint32_t FMSSTART;                    
  volatile uint32_t FMSSTOP;                     
  volatile const  uint32_t RESERVED2[1];
  volatile const  uint32_t FMSW0;                       
  volatile const  uint32_t FMSW1;                       
  volatile const  uint32_t FMSW2;                       
  volatile const  uint32_t FMSW3;                       
  volatile const  uint32_t RESERVED3[1001];
  volatile const  uint32_t FMSTAT;                      
  volatile const  uint32_t RESERVED4[1];
  volatile uint32_t FMSTATCLR;                   
} LPC_FLASHCTRL_Type;









 

typedef struct {                             
  volatile uint32_t CR0;                         
  volatile uint32_t CR1;                         
  volatile uint32_t DR;                          
  volatile const  uint32_t SR;                          
  volatile uint32_t CPSR;                        
  volatile uint32_t IMSC;                        
  volatile const  uint32_t RIS;                         
  volatile const  uint32_t MIS;                         
  volatile uint32_t ICR;                         
} LPC_SSPx_Type;










 

typedef struct {                             
  volatile uint32_t RESET_PIO0_0;                
  volatile uint32_t PIO0_1;                      
  volatile uint32_t PIO0_2;                      
  volatile uint32_t PIO0_3;                      
  volatile uint32_t PIO0_4;                      
  volatile uint32_t PIO0_5;                      
  volatile uint32_t PIO0_6;                      
  volatile uint32_t PIO0_7;                      
  volatile uint32_t PIO0_8;                      
  volatile uint32_t PIO0_9;                      
  volatile uint32_t SWCLK_PIO0_10;               
  volatile uint32_t TDI_PIO0_11;                 
  volatile uint32_t TMS_PIO0_12;                 
  volatile uint32_t TDO_PIO0_13;                 
  volatile uint32_t TRST_PIO0_14;                
  volatile uint32_t SWDIO_PIO0_15;               
  volatile uint32_t PIO0_16;                     
  volatile uint32_t PIO0_17;                     
  volatile uint32_t PIO0_18;                     
  volatile uint32_t PIO0_19;                     
  volatile uint32_t PIO0_20;                     
  volatile uint32_t PIO0_21;                     
  volatile uint32_t PIO0_22;                     
  volatile uint32_t PIO0_23;                     
  volatile uint32_t PIO1_0;                  
  volatile uint32_t PIO1_1;         
  volatile uint32_t PIO1_2;       
  volatile uint32_t PIO1_3;      
  volatile uint32_t PIO1_4;                  
  volatile uint32_t PIO1_5;                      
  volatile uint32_t PIO1_6;     
  volatile uint32_t PIO1_7;       
  volatile uint32_t PIO1_8;                  
  volatile uint32_t PIO1_9;        
  volatile uint32_t PIO1_10;        
  volatile uint32_t PIO1_11;       
  volatile uint32_t PIO1_12;                 
  volatile uint32_t PIO1_13;                     
  volatile uint32_t PIO1_14;                     
  volatile uint32_t PIO1_15;                     
  volatile uint32_t PIO1_16;                     
  volatile uint32_t PIO1_17;
  volatile uint32_t PIO1_18;
  volatile uint32_t PIO1_19;                     
  volatile uint32_t PIO1_20;                     
  volatile uint32_t PIO1_21;                     
  volatile uint32_t PIO1_22;                     
  volatile uint32_t PIO1_23;                     
  volatile uint32_t PIO1_24;                     
  volatile uint32_t PIO1_25;                     
  volatile uint32_t PIO1_26;                     
  volatile uint32_t PIO1_27;                     
  volatile uint32_t PIO1_28;                     
  volatile uint32_t PIO1_29;                     
  volatile uint32_t PIO1_30;
  volatile uint32_t PIO1_31;                     
} LPC_IOCON_Type;









 

typedef struct {                             
  volatile uint32_t SYSMEMREMAP;                 
  volatile uint32_t PRESETCTRL;                  
  volatile uint32_t SYSPLLCTRL;                  
  volatile const  uint32_t SYSPLLSTAT;                  
  volatile uint32_t USBPLLCTRL;                  
  volatile const  uint32_t USBPLLSTAT;                  
  volatile const  uint32_t RESERVED0[2];
  volatile uint32_t SYSOSCCTRL;                  
  volatile uint32_t WDTOSCCTRL;                  
  volatile const  uint32_t RESERVED1[2];
  volatile uint32_t SYSRSTSTAT;                  
  volatile const  uint32_t RESERVED2[3];
  volatile uint32_t SYSPLLCLKSEL;                
  volatile uint32_t SYSPLLCLKUEN;                
  volatile uint32_t USBPLLCLKSEL;                
  volatile uint32_t USBPLLCLKUEN;                
  volatile const  uint32_t RESERVED3[8];
  volatile uint32_t MAINCLKSEL;                  
  volatile uint32_t MAINCLKUEN;                  
  volatile uint32_t SYSAHBCLKDIV;                
  volatile const  uint32_t RESERVED4[1];
  volatile uint32_t SYSAHBCLKCTRL;               
  volatile const  uint32_t RESERVED5[4];
  volatile uint32_t SSP0CLKDIV;                  
  volatile uint32_t UARTCLKDIV;                  
  volatile uint32_t SSP1CLKDIV;                  
  volatile const  uint32_t RESERVED6[8];
  volatile uint32_t USBCLKSEL;                   
  volatile uint32_t USBCLKUEN;                   
  volatile uint32_t USBCLKDIV;                   
  volatile const  uint32_t RESERVED7[5];
  volatile uint32_t CLKOUTSEL;                   
  volatile uint32_t CLKOUTUEN;                   
  volatile uint32_t CLKOUTDIV;                   
  volatile const  uint32_t RESERVED8[5];
  volatile const  uint32_t PIOPORCAP0;                  
  volatile const  uint32_t PIOPORCAP1;                  
  volatile const  uint32_t RESERVED9[18];
  volatile uint32_t BODCTRL;                     
  volatile uint32_t SYSTCKCAL;                   
  volatile const  uint32_t RESERVED10[6];
  volatile uint32_t IRQLATENCY;                  
  volatile uint32_t NMISRC;                      
  volatile uint32_t PINTSEL[8];                  
  volatile uint32_t USBCLKCTRL;                  
  volatile const  uint32_t USBCLKST;                    
  volatile const  uint32_t RESERVED11[25];
  volatile uint32_t STARTERP0;                   
  volatile const  uint32_t RESERVED12[3];
  volatile uint32_t STARTERP1;                   
  volatile const  uint32_t RESERVED13[6];
  volatile uint32_t PDSLEEPCFG;                  
  volatile uint32_t PDAWAKECFG;                  
  volatile uint32_t PDRUNCFG;                    
  volatile const  uint32_t RESERVED14[110];
  volatile const  uint32_t DEVICE_ID;                   
} LPC_SYSCON_Type;









 

typedef struct {                             
  volatile uint32_t ISEL;                        
  volatile uint32_t IENR;                        
  volatile uint32_t SIENR;                       
  volatile uint32_t CIENR;                       
  volatile uint32_t IENF;                        
  volatile uint32_t SIENF;                       
  volatile uint32_t CIENF;                       
  volatile uint32_t RISE;                        
  volatile uint32_t FALL;                        
  volatile uint32_t IST;                         
} LPC_GPIO_PIN_INT_Type;









 

typedef struct {                             
  volatile uint32_t CTRL;                        
  volatile const  uint32_t RESERVED0[7];
  volatile uint32_t PORT_POL[2];                 
  volatile const  uint32_t RESERVED1[6];
  volatile uint32_t PORT_ENA[2];                 
} LPC_GPIO_GROUP_INTx_Type;










 

typedef struct {                             
  volatile uint32_t DEVCMDSTAT;                  
  volatile uint32_t INFO;                        
  volatile uint32_t EPLISTSTART;                 
  volatile uint32_t DATABUFSTART;                
  volatile uint32_t LPM;                         
  volatile uint32_t EPSKIP;                      
  volatile uint32_t EPINUSE;                     
  volatile uint32_t EPBUFCFG;                    
  volatile uint32_t INTSTAT;                     
  volatile uint32_t INTEN;                       
  volatile uint32_t INTSETSTAT;                  
  volatile uint32_t INTROUTING;                  
  volatile const  uint32_t RESERVED0[1];
  volatile const  uint32_t EPTOGGLE;                    
} LPC_USB_Type;









 

typedef struct {                            
  union {
    struct {
      volatile uint8_t B0[32];                        
      volatile uint8_t B1[32];                        
    };
    volatile uint8_t B[64];                        
  };
  volatile const  uint32_t RESERVED0[1008];
  union {
    struct {
      volatile uint32_t W0[32];                       
      volatile uint32_t W1[32];                       
    };
    volatile uint32_t W[64];                        
  };
       uint32_t RESERVED1[960];
  volatile uint32_t DIR[2];			 
       uint32_t RESERVED2[30];
  volatile uint32_t MASK[2];		 
       uint32_t RESERVED3[30];
  volatile uint32_t PIN[2];			 
       uint32_t RESERVED4[30];
  volatile uint32_t MPIN[2];		 
       uint32_t RESERVED5[30];
  volatile uint32_t SET[2];			 
       uint32_t RESERVED6[30];
  volatile  uint32_t CLR[2];			 
       uint32_t RESERVED7[30];
  volatile  uint32_t NOT[2];			 
} LPC_GPIO_Type;



  #pragma no_anon_unions







#line 632 "..\\Common\\inc\\lpc11Uxx.h"






#line 657 "..\\Common\\inc\\lpc11Uxx.h"


   
   
   






#line 2 "..\\User\\SPI_FLASH.c"
#line 1 "..\\Common\\inc\\type.h"


















 



 
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;













#line 3 "..\\User\\SPI_FLASH.c"
#line 1 "..\\Common\\inc\\gpio.h"


















 



#line 31 "..\\Common\\inc\\gpio.h"







void FLEX_INT0_IRQHandler(void);
void FLEX_INT1_IRQHandler(void);
void FLEX_INT2_IRQHandler(void);
void FLEX_INT3_IRQHandler(void);
void FLEX_INT4_IRQHandler(void);
void FLEX_INT5_IRQHandler(void);
void FLEX_INT6_IRQHandler(void);
void FLEX_INT7_IRQHandler(void);
void GINT0_IRQHandler(void);
void GINT1_IRQHandler(void);
void GPIOInit( void );
void GPIOSetFlexInterrupt( uint32_t channelNum, uint32_t portNum, uint32_t bitPosi,
		uint32_t sense, uint32_t event );
void GPIOFlexIntEnable( uint32_t channelNum, uint32_t event );
void GPIOFlexIntDisable( uint32_t channelNum, uint32_t event );
uint32_t GPIOFlexIntStatus( uint32_t channelNum );
void GPIOFlexIntClear( uint32_t channelNum );
void GPIOSetGroupedInterrupt( uint32_t groupNum, uint32_t *bitPattern, uint32_t logic,
		uint32_t sense, uint32_t *eventPattern );
uint32_t GPIOGetPinValue( uint32_t portNum, uint32_t bitPosi );
void GPIOSetBitValue( uint32_t portNum, uint32_t bitPosi, uint32_t bitVal );
void GPIOSetDir( uint32_t portNum, uint32_t bitPosi, uint32_t dir );




 
#line 4 "..\\User\\SPI_FLASH.c"
#line 1 "..\\userinc\\SPI_Flash.h"



#line 13 "..\\userinc\\SPI_Flash.h"

                                                     
#line 33 "..\\userinc\\SPI_Flash.h"

extern		void		SPI_FLASH_Init(void);
extern		void 		SPI_Flash_Erase_Sector(uint32_t Dst_Addr);
extern		void 		SPI_Flash_Erase_Chip(void);
extern		void 		Flash_WriteByte(uint8_t* pBuffer,uint32_t WriteAddr);
extern		void 		SPI_Flash_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
extern		void 		AutoAddressIncrement_WordProgramB(uint8_t state,uint8_t Byte1, uint8_t Byte2);
extern		void 		AutoAddressIncrement_WordProgramA(uint8_t Byte1, uint8_t Byte2, uint32_t Addr);
extern		void 		SPI_Flash_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);
extern		uint16_t 	SPI_Flash_ReadID(void);
extern		void 		SPI_FLASH_Write_SR(uint8_t sr);
extern		uint8_t 	SPI_Flash_ReadSR(void);
extern		void		SPI_Flash_Config(void);
extern		uint8_t		SPI_Flash_ReadWriteData(uint8_t Data);


extern		void	SPI_FLASH_Write_Disable(void);
extern		void	SPI_FLASH_Write_Enable(void);
extern		void	SST25V_DBSY(void);
extern		void	SST25V_EBSY(void);	





#line 5 "..\\User\\SPI_FLASH.c"








 
void	SPI_Flash_Config(void)
{
	((LPC_IOCON_Type *) (0x40044000))->PIO1_20				&=~0x07;  	    
	((LPC_IOCON_Type *) (0x40044000))->PIO1_21				&=~0x07;  	    
	((LPC_IOCON_Type *) (0x40044000))->PIO1_22				&=~0x07;  	    

	((LPC_IOCON_Type *) (0x40044000))->PIO1_20				|=0x02;  	    
	((LPC_IOCON_Type *) (0x40044000))->PIO1_21				|=0x02;  	    
	((LPC_IOCON_Type *) (0x40044000))->PIO1_22				|=0x02;  	    
									  									   	
	((LPC_SYSCON_Type *) (0x40048000))->PRESETCTRL 			|=(0x01 << 2);			
	((LPC_SYSCON_Type *) (0x40048000))->SYSAHBCLKCTRL 		|=(0X01 << 18);			
	((LPC_SYSCON_Type *) (0x40048000))->SSP1CLKDIV			=0x02;					
	((LPC_SSPx_Type *) (0x40058000))->CR0					=	(0x07 << 0)	   		
									  |	(0x00 << 4)			
									  | (0x00 << 6)			
									  | (0x00 << 7)			
									  | (0x07 << 8);		
	
	((LPC_SSPx_Type *) (0x40058000))->CR1					=	(0X00 << 0)		    
									  | (0X01 << 1)		    
									  | (0X00 << 2)		    
									  | (0X00 << 3);	    
	((LPC_SSPx_Type *) (0x40058000))->CPSR					=0X01;				    
	((LPC_SSPx_Type *) (0x40058000))->ICR					=0X03;				    
	((LPC_SSPx_Type *) (0x40058000))->IMSC					=0x00;				    
}
void	SPI_FLASH_Init(void)
{
	SPI_Flash_Config();
	GPIOSetDir( 1, 23, 1 );
	SPI_Flash_ReadWriteData(0x02);  	
	SST25V_DBSY();
}







 
uint8_t	SPI_Flash_ReadWriteData(uint8_t Data)
{	
	((LPC_SSPx_Type *) (0x40058000))->DR	=Data;
	while(((LPC_SSPx_Type *) (0x40058000)) ->SR & (1<<4));						
	return 	(((LPC_SSPx_Type *) (0x40058000))->DR);	
}









 
uint8_t SPI_Flash_ReadSR(void)  
{ 
		uint8_t byte=0; 
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);
		SPI_Flash_ReadWriteData( 0x05 );    
		byte=SPI_Flash_ReadWriteData(0Xff);                
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);     
		return byte;  
}



void SPI_FLASH_Write_SR(uint8_t sr)  
{  
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);
		SPI_Flash_ReadWriteData(0x50);  
		SPI_Flash_ReadWriteData(0x01);   	  
		SPI_Flash_ReadWriteData(sr);                          
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23); 
} 
                            

uint16_t SPI_Flash_ReadID(void)
{
		uint16_t Temp = 0;
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);      				                           
		SPI_Flash_ReadWriteData(0x90);					 	
		SPI_Flash_ReadWriteData(0x00);             		 	
		SPI_Flash_ReadWriteData(0x00);             
		SPI_Flash_ReadWriteData(0x00);		                              					   
		Temp	=SPI_Flash_ReadWriteData(0xFF)<<8;  	
		Temp   +=SPI_Flash_ReadWriteData(0xFF);
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);                                     
		return Temp;
}





void SPI_Flash_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)  
{
		uint16_t i;
		while ((SPI_Flash_ReadSR()&0x01)==0x01);
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);                                                                                                                 
		SPI_Flash_ReadWriteData(0x03);         		  
		SPI_Flash_ReadWriteData((uint8_t)((ReadAddr)>>16));   	  
		SPI_Flash_ReadWriteData((uint8_t)((ReadAddr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)ReadAddr);
		for(i=0;i<NumByteToRead;i++)
		{
		  *pBuffer++	=SPI_Flash_ReadWriteData(0XFF);   		  
		}
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);            
} 

void AutoAddressIncrement_WordProgramA(uint8_t Byte1, uint8_t Byte2, uint32_t Addr)
{
		SPI_FLASH_Write_Enable();
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);
		SPI_Flash_ReadWriteData(0xAD);				
		SPI_Flash_ReadWriteData((uint8_t)(Addr >> 16));		   
		SPI_Flash_ReadWriteData((uint8_t)(Addr >> 8));
		SPI_Flash_ReadWriteData((uint8_t)(Addr & 0xFF));       
		SPI_Flash_ReadWriteData(Byte1);						   
		SPI_Flash_ReadWriteData(Byte2);
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
		while ((SPI_Flash_ReadSR()&0x01)==0x01);
}


void AutoAddressIncrement_WordProgramB(uint8_t state,uint8_t Byte1, uint8_t Byte2)
{
		SPI_FLASH_Write_Enable();
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);
		SPI_Flash_ReadWriteData(0xAD);
		SPI_Flash_ReadWriteData(Byte1);
		SPI_Flash_ReadWriteData(Byte2);
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
		while ((SPI_Flash_ReadSR()&0x01)==0x01);
		if(state==1)
		{
		  SPI_FLASH_Write_Disable();
		}
		while ((SPI_Flash_ReadSR()&0x01)==0x01);
}






void SPI_Flash_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
		uint16_t i,temp;
		uint32_t secpos;
		uint16_t secoff;
		uint16_t secremain;     
		
		secpos		=	WriteAddr	/	4096;      		
		secoff		=	WriteAddr	%	4096;             
		secremain	=	4096		-	secoff;         
		if(NumByteToWrite<secremain)   temp=1;          
		else                                            
		{
		  i=NumByteToWrite-secremain;                   
		  if(i%4096==0)	temp=i/4096+1;
		  else			temp=i/4096+2;
		}
		for(i=0;i<temp;i++)
		{
		  SPI_Flash_Erase_Sector((secpos+i)*4096);      
		}
  
		if(NumByteToWrite%2==0)		temp=NumByteToWrite/2-1;
		else						temp=NumByteToWrite/2;
		AutoAddressIncrement_WordProgramA(pBuffer[0], pBuffer[1],WriteAddr );        
		for(i=1;i<temp;i++)	AutoAddressIncrement_WordProgramB(0,pBuffer[2*i], pBuffer[2*i+1]);
		if(NumByteToWrite%2==0)	AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-2], pBuffer[NumByteToWrite-1]);   
		else	AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-1],0);                                         

}



void Flash_WriteByte(uint8_t* pBuffer,uint32_t WriteAddr)
{
		uint32_t secpos;
		secpos	=WriteAddr/4096;                                
		SPI_Flash_Erase_Sector(secpos);                         
		SPI_FLASH_Write_Enable();                              	
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);
		SPI_Flash_ReadWriteData(0x02 );       		
		SPI_Flash_ReadWriteData((uint8_t)((WriteAddr)>>16));  	
		SPI_Flash_ReadWriteData((uint8_t)((WriteAddr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)WriteAddr);                 
		SPI_Flash_ReadWriteData(pBuffer[0]);					
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
		while ((SPI_Flash_ReadSR()&0x01)==0x01);                                  
}







void SPI_Flash_Erase_Chip(void)  
{                                            
		SPI_FLASH_Write_Enable();                          	
		while ((SPI_Flash_ReadSR()&0x01)==0x01); 
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23); 
		SPI_Flash_ReadWriteData(0xC7);       	
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
		while ((SPI_Flash_ReadSR()&0x01)==0x01);                              
}



void SPI_Flash_Erase_Sector(uint32_t Dst_Addr)  
{  
		SPI_FLASH_Write_Enable();                             	
		while ((SPI_Flash_ReadSR()&0x01)==0x01);
		(((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23);  
		SPI_Flash_ReadWriteData(0x20);       
		SPI_Flash_ReadWriteData((uint8_t)((Dst_Addr)>>16));     
		SPI_Flash_ReadWriteData((uint8_t)((Dst_Addr)>>8));  
		SPI_Flash_ReadWriteData((uint8_t)Dst_Addr);
		(((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);              
		while ((SPI_Flash_ReadSR()&0x01)==0x01);                                  
} 

void	SST25V_EBSY(void)
{		 
		 (((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23); 
		 SPI_Flash_ReadWriteData( 0x70);
		 (((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
}
void	SST25V_DBSY(void)
{		 
		 (((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23); 
		 SPI_Flash_ReadWriteData( 0x80);
		 (((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
}
void	SPI_FLASH_Write_Enable(void)
{		 
		 (((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23); 
		 SPI_Flash_ReadWriteData( 0x06);
		 (((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
}
void	SPI_FLASH_Write_Disable(void)
{		 
		 (((LPC_GPIO_Type *) (0x50000000))->CLR[1] = 1<<23); 
		 SPI_Flash_ReadWriteData( 0x04);
		 (((LPC_GPIO_Type *) (0x50000000))->SET[1] = 1<<23);
}






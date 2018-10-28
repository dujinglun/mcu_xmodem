#line 1 "..\\User\\CV520.c"
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
        (((priority << (8 - 3)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IPR[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IPR[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 3)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}













 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 3)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IPR[( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 3)));  }  
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
  NVIC_SetPriority (SysTick_IRQn, (1<<3) - 1);   
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


   
   
   






#line 2 "..\\User\\CV520.c"
#line 1 "..\\Common\\inc\\type.h"


















 



 
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;













#line 3 "..\\User\\CV520.c"
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




 
#line 4 "..\\User\\CV520.c"
#line 1 "..\\userinc\\CV520.h"




























void		CV520_HWReset(void);
void		CV520_IOConfig(void);
void		CV520_SPI_Config(void);
uint8_t		CV520_SPI_SendData(uint8_t Data);

void	CV520_Test(void);
void	CV520_Init(void);
void	CV520_PcdReset(void);
uint8_t CV520_PcdSelect(uint8_t *pSnr);
uint8_t CV520_PcdAnticoll(uint8_t *pSnr);
uint8_t	CV520_PcdRequest(uint8_t Req_Code,uint8_t *pTagType);
uint8_t CV520_PcdValue(uint8_t DD_mode,uint8_t Addr,uint8_t *pValue);
uint8_t CV520_PcdAuthState(uint8_t Auth_mode,uint8_t Addr,uint8_t *pKey,uint8_t *pSnr);
uint8_t CV520_PcdBakValue(uint8_t DstAddr,uint8_t SrcAddr);
uint8_t CV520_PcdWrite(uint8_t Addr,uint8_t *pData);
uint8_t CV520_PcdRead(uint8_t Addr,uint8_t *pData);
uint8_t CV520_PcdHalt(void);

void	CV520_FormatData2Buff( int_fast32_t Data , uint8_t BKAddr , uint8_t *Buff);
uint8_t	CV520_FormatBuff2Data(int_fast32_t *Data , uint8_t *Addr);



typedef		struct
{
  		uint8_t		Cmd;
		uint8_t		WriteToM1[18];
		uint8_t		WriteByte;
		uint8_t		CRCByte;
		uint8_t		ReadFromM1[18];
		uint8_t		ReadBit;
}PCD_DataFormat;
typedef		struct
{
        
        
        
        
        
		uint8_t		TagType[2];		
		uint8_t		SerialNum[4];	
		uint8_t		SecNum;			
		uint8_t		BankNum;		
		uint8_t		PhysicsAddr;	
		uint8_t		SecKeyA[6];		
		uint8_t		VisitCtr[4];	
		uint8_t		SecKeyB[6];		
		uint8_t		DataBuff[16];	
}Card_Infor;


#line 87 "..\\userinc\\CV520.h"


#line 102 "..\\userinc\\CV520.h"






#line 124 "..\\userinc\\CV520.h"

#line 141 "..\\userinc\\CV520.h"

#line 158 "..\\userinc\\CV520.h"

#line 175 "..\\userinc\\CV520.h"








#line 5 "..\\User\\CV520.c"
#line 1 "..\\Common\\inc\\uart.h"


















 

















#line 45 "..\\Common\\inc\\uart.h"



 
#line 55 "..\\Common\\inc\\uart.h"

void ModemInit( void );
void UARTInit(uint32_t Baudrate);
void UART_IRQHandler(void);
void UARTSend(uint8_t *BufferPtr, uint32_t Length);
void print_string( uint8_t *str_ptr );
uint8_t get_key( void );
void	Myprintf(uint8_t *Str,uint8_t Num);




 
#line 6 "..\\User\\CV520.c"







							
extern	void delaySysTick(uint32_t tick);
PCD_DataFormat		M1_PCDData;







 
void	CV520_IOConfig(void)
{
	GPIOSetDir(1   	,28 		, 0);
	GPIOSetDir(1   	,31 		, 0);
	GPIOSetDir(0   	,7 	, 1);
	GPIOSetDir(0   	,8 		, 0);
	GPIOSetDir(1   	,19 				, 1);
}







 
void	CV520_HWReset(void)
{
		GPIOSetBitValue( 0,7,0 );			delaySysTick(5000);
		GPIOSetBitValue( 0,7,1 );			delaySysTick(5000);
}







 
void	CV520_SPI_Config(void)
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
	((LPC_SSPx_Type *) (0x40058000))->CPSR					=0X02;				    
	((LPC_SSPx_Type *) (0x40058000))->ICR					=0X03;				    
	((LPC_SSPx_Type *) (0x40058000))->IMSC					=0x00;				    
}







 
uint8_t	CV520_SPI_SendData(uint8_t Data)
{	
	((LPC_SSPx_Type *) (0x40058000))->DR	=Data;
	while(((LPC_SSPx_Type *) (0x40058000)) ->SR & (1<<4));						
	return 	(((LPC_SSPx_Type *) (0x40058000))->DR);	
}









 
void	CV520_WriteReg(uint8_t Addr,uint8_t Value)
{
	 uint8_t	AddrV;
	 AddrV		=(Addr << 1) & 0x7e;
	 GPIOSetBitValue(1 ,19 , 0);
	 CV520_SPI_SendData( AddrV );
	 CV520_SPI_SendData( Value );
	 GPIOSetBitValue(1 ,19 , 1);	
}







 
uint8_t	CV520_ReadReg(uint8_t Addr)
{
	uint8_t		AddrV , RegValue;
	AddrV		=((Addr  << 1)& 0xfe) | 0x80;
	GPIOSetBitValue(1 ,19 , 0);
	CV520_SPI_SendData( AddrV );
	RegValue	=CV520_SPI_SendData( 0xff );
	GPIOSetBitValue(1 ,19 , 1);
	
	return  RegValue;
}







 
void	CV520_SetBitMask(uint8_t Reg, uint8_t Mask)
{
 	char 	temp;
	temp	=CV520_ReadReg( Reg );
	CV520_WriteReg( Reg , temp | Mask);
}







 
void	CV520_ClrBitMask(uint8_t Reg, uint8_t Mask)
{
 	char 	temp;
	Mask	=0xff ^ Mask;
	temp	=CV520_ReadReg( Reg );
	CV520_WriteReg( Reg , temp & Mask);
}







 
void	CV520_PcdAntennaOn(void)
{
	uint8_t	temp;
	temp	=CV520_ReadReg( 0x14 );
	if( !(temp & 0x03))
	{
		CV520_SetBitMask(0x14,0x03);
	}
}







 
void	CV520_PcdAnternaOff(void)
{
		CV520_ClrBitMask(0x14,0x03);		delaySysTick(5000);
}







 
#line 217 "..\\User\\CV520.c"







 
void	CV520_PcdConfigISOType(uint8_t Type)
{
   if (Type == 'A')                     
   { 
       CV520_ClrBitMask(0x08,0x08);
 








 
       CV520_WriteReg(0x11,0x3D);		




 
       CV520_WriteReg(0x17,0x86);		
       CV520_WriteReg(0x18,0x84);
       CV520_WriteReg(0x19,0x4D);      

       CV520_WriteReg(0x24,0x13);	
       CV520_WriteReg(0x26,0x7F);   	



 
   	   CV520_WriteReg(0x2D,30);
	   CV520_WriteReg(0x2C,0);
       CV520_WriteReg(0x2A,0x8D);
	   CV520_WriteReg(0x2B,0x3E);
	   delaySysTick(5000);
       CV520_PcdAntennaOn();
   }
}







 
void	CV520_PcdReset(void)
{
		CV520_HWReset();
		CV520_WriteReg(0x01		,0x0F);
		delaySysTick(5000);
    	CV520_WriteReg(0x11			,0x3D);      
    	CV520_WriteReg(0x2D		,30);           
    	CV520_WriteReg(0x2C		,0);
    	CV520_WriteReg(0x2A			,0x8D);
    	CV520_WriteReg(0x2B	,0x3E);
    	CV520_WriteReg(0x15			,0x40);
}







 
void	CV520_Init(void)
{
	CV520_IOConfig();
	CV520_SPI_Config();
	CV520_PcdReset();
	CV520_PcdAnternaOff();
	CV520_PcdAntennaOn();

}







 
void CV520_CreateCRC(uint8_t	*OutCRC)
{
    uint8_t i,n;
    CV520_ClrBitMask(0x05		,0x04);
    CV520_WriteReg(0x01		,0x00);
    CV520_SetBitMask(0x0A	,0x80);
    for (i=0; i<M1_PCDData.CRCByte; i++)	
	{
	   CV520_WriteReg(0x09	,M1_PCDData.WriteToM1[i]);
	}
       CV520_WriteReg(0x01	,0x03);
    i = 25;
    do 
    {
        n = CV520_ReadReg(0x05);
		delaySysTick(2);
        i--;
    }while((i!=0) && !(n&0x04));
	*OutCRC++	=CV520_ReadReg( 0x22 );
	*OutCRC 	=CV520_ReadReg( 0x21 );
}







 
uint8_t		CV520_PcdComCV520(void)
{
  	    uint8_t	Status	=2;
		uint8_t irqEn=0,  waitFor=0, lastBits, n ,i,Retry;
		if( M1_PCDData.Cmd == 0x0E )
		{
			 irqEn   = 0x12;
          	 waitFor = 0x10;
		}
		else if( M1_PCDData.Cmd == 0x0C )
		{
			 irqEn   = 0x77;
          	 waitFor = 0x30;
		}
		else	return Status;
		CV520_WriteReg	(0x02		,	irqEn|0x80 );
    	CV520_ClrBitMask(0x04		,	0x80	   );
    	CV520_WriteReg  (0x01		,	0x00   );
    	CV520_SetBitMask(0x0A	,	0x80);
		for(i=0;i<M1_PCDData.WriteByte ; i++)
		{
			 CV520_WriteReg(0x09 , 	M1_PCDData.WriteToM1[i]);
		}
		CV520_WriteReg(0x01, 	M1_PCDData.Cmd);
		if( M1_PCDData.Cmd == 0x0C )
		{
			 CV520_SetBitMask(0x0D,0x80);
		}
		Retry	=25;
		do
		{
			n	=CV520_ReadReg(0x04);
			delaySysTick(2);
			Retry--;
		}while((Retry!=0) && !(n&0x01) && !(n&waitFor));
		CV520_ClrBitMask(0x0D		,0x80);
		
		if( Retry !=0 )
		{
		  	if( !(  CV520_ReadReg(0x06) & 0x1B ))
			{
			  	Status	=0;
				if( n & irqEn & 0x01)
				{
					Status	=1;
				}
				if( M1_PCDData.Cmd == 0x0C)
				{
					n			=  CV520_ReadReg( 0x0A );
					lastBits	=  CV520_ReadReg( 0x0C ) & 0x07;
					if( lastBits )		M1_PCDData.ReadBit	=(n-1)*8 + lastBits;
					else				M1_PCDData.ReadBit	=n*8;
					if( n== 0)			n=1;
					if( n > 18)		n=18;
					for(i=0;i<n;i++)
					{
						 M1_PCDData.ReadFromM1[i]	=CV520_ReadReg( 0x09 );
					}
				}				
			}
			else
			{
				Status	=2;
			}
		}
			CV520_SetBitMask(0x0C,0x80);           
   			CV520_WriteReg(0x01,0x00);
			return Status;		
}


















 
uint8_t	CV520_PcdRequest(uint8_t Req_Code,uint8_t *pTagType)
{
   uint8_t  Status = 0;  
   CV520_ClrBitMask(0x08			,0x08);
   CV520_WriteReg(0x0D			,0x07);
   CV520_SetBitMask(0x14		,0x03); 

   M1_PCDData.Cmd			=0x0C;
   M1_PCDData.WriteToM1[0] 	=Req_Code;
   M1_PCDData.WriteByte		=1;
   M1_PCDData.ReadBit		=0;
   Status=CV520_PcdComCV520();
   if((Status == 0) && (M1_PCDData.ReadBit == 0x10))
   {    
       *pTagType++     = M1_PCDData.ReadFromM1[0];
       *pTagType	   = M1_PCDData.ReadFromM1[1];
   }
   else	Status = 2;
   return Status;
}








 
uint8_t CV520_PcdAnticoll(uint8_t *pSnr)
{
    uint8_t Status,i,snr_check=0;
    CV520_ClrBitMask(0x08,		0x08);
    CV520_WriteReg(0x0D,		0x00);
    CV520_ClrBitMask(0x0E,			0x80);
 
	M1_PCDData.Cmd				=0x0C;
    M1_PCDData.WriteToM1[0] 	=0x93;
	M1_PCDData.WriteToM1[1] 	=0x20;
    M1_PCDData.WriteByte		=2;
    M1_PCDData.ReadBit			=0;
	Status=CV520_PcdComCV520();
    if (Status == 0)
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = M1_PCDData.ReadFromM1[i];
             snr_check ^= M1_PCDData.ReadFromM1[i];
         }
         if (snr_check != M1_PCDData.ReadFromM1[i])
         {   
		 	Status = 2;
	     }
    }   
    CV520_SetBitMask(0x0E,			0x80);
    return Status;
}








 
uint8_t CV520_PcdSelect(uint8_t *pSnr)
{
    uint8_t Status	,i;
	M1_PCDData.Cmd				=0x0C;
    M1_PCDData.WriteToM1[0] 	=0x93;
	M1_PCDData.WriteToM1[1] 	=0x70;
	M1_PCDData.WriteToM1[6] 	=0x00;
	M1_PCDData.WriteToM1[7] 	=0x00;
	M1_PCDData.WriteToM1[8] 	=0x00;
    M1_PCDData.WriteByte		=9;
	M1_PCDData.CRCByte			=7;
    M1_PCDData.ReadBit			=0;
    for (i=0; i<4; i++)
    {
    	M1_PCDData.WriteToM1[i+2] = *(pSnr+i);
    	M1_PCDData.WriteToM1[6]  ^= *(pSnr+i);
    }
	CV520_CreateCRC(&M1_PCDData.WriteToM1[7]);  
    Status=CV520_PcdComCV520();   
    if ((Status == 0) && (M1_PCDData.ReadBit == 0x18))	Status = 0;  
    else 													Status = 2;    
    return Status;
}













 
uint8_t CV520_PcdAuthState(uint8_t Auth_mode,uint8_t Addr,uint8_t *pKey,uint8_t *pSnr)
{
    uint8_t Status , i;
	M1_PCDData.Cmd				=0x0E;
	M1_PCDData.WriteToM1[0]		=Auth_mode;
	M1_PCDData.WriteToM1[1]		=Addr;
	M1_PCDData.WriteByte		=12;
    M1_PCDData.ReadBit			=0;
    for (i=2; i<8; i++)			M1_PCDData.WriteToM1[i]	= *pKey++;
    for (i=8; i<12; i++)	    M1_PCDData.WriteToM1[i]	= *pSnr++; 
    Status=CV520_PcdComCV520();
	i	=CV520_ReadReg( 0x08 );  
    if((Status != 0) || (!(i & 0x08)))
    {   
		Status = 2;
	}    
    return Status;
}









 
uint8_t CV520_PcdWrite(uint8_t Addr,uint8_t *pData)
{
    uint8_t Status , i;

	M1_PCDData.Cmd				=0x0C;
	M1_PCDData.WriteToM1[0]		=0xA0;
	M1_PCDData.WriteToM1[1]		=Addr;
	M1_PCDData.WriteByte		=4;
	M1_PCDData.CRCByte			=2;
    M1_PCDData.ReadBit			=0;	
	CV520_CreateCRC(&M1_PCDData.WriteToM1[2]);
	Status=CV520_PcdComCV520();
    if ((Status != 0) || (M1_PCDData.ReadBit != 4) 
	     || ((M1_PCDData.ReadFromM1[0] & 0x0F) != 0x0A))
	{
		 		Status = 2;
	}           
    else if (Status == 0)
    {
		M1_PCDData.Cmd				=0x0C;
		M1_PCDData.CRCByte			=16;
		M1_PCDData.WriteByte		=18;
    	M1_PCDData.ReadBit			=0;
        for (i=0; i<16; i++) 	M1_PCDData.WriteToM1[i]	=*pData++;
		CV520_CreateCRC(&M1_PCDData.WriteToM1[16]);		
		Status=CV520_PcdComCV520();
        if ((Status != 0) || (M1_PCDData.ReadBit != 4)
		    || ((M1_PCDData.ReadFromM1[0] & 0x0F) != 0x0A))
		{
				 Status = 2; 
		}	
    }
    
    return Status;
}










 
uint8_t CV520_PcdValue(uint8_t DD_mode,uint8_t Addr,uint8_t *pValue)
{
    uint8_t Status , i;
   	M1_PCDData.Cmd				=0x0C;
	M1_PCDData.WriteToM1[0]		=DD_mode;
	M1_PCDData.WriteToM1[1]		=Addr;
	M1_PCDData.CRCByte			=2;
	M1_PCDData.WriteByte		=4;
    M1_PCDData.ReadBit			=0;
	CV520_CreateCRC(&M1_PCDData.WriteToM1[2]);	
	Status=CV520_PcdComCV520();
    if ((Status != 0) || (M1_PCDData.ReadBit != 4) 
	    || ((M1_PCDData.ReadFromM1[0] & 0x0F) != 0x0A))	Status = 2;         
    if (Status == 0)
    {         
		 M1_PCDData.Cmd				=0x0C;
		 M1_PCDData.CRCByte			=4;
		 M1_PCDData.WriteByte		=6;
		 M1_PCDData.ReadBit			=0;
		 for (i=0; i<16; i++) 	M1_PCDData.WriteToM1[i]	=*pValue++;
         CV520_CreateCRC(&M1_PCDData.WriteToM1[4]);		 
         Status=CV520_PcdComCV520();
         if(Status != 2)		Status = 0;
    }    
    if (Status == 0)
    {
		M1_PCDData.Cmd				=0x0C;
		M1_PCDData.WriteToM1[0]		=0xB0;
		M1_PCDData.WriteToM1[1]		=Addr;
		M1_PCDData.CRCByte			=2;
		M1_PCDData.WriteByte		=4;
		M1_PCDData.ReadBit			=0;
        CV520_CreateCRC(&M1_PCDData.WriteToM1[2]);     	
        Status=CV520_PcdComCV520();
    	if ((Status != 0) || (M1_PCDData.ReadBit != 4) 
		   || ((M1_PCDData.ReadFromM1[0] & 0x0F) != 0x0A))	Status = 2;         
    }
    return Status;
}









 
uint8_t CV520_PcdBakValue(uint8_t DstAddr,uint8_t SrcAddr)
{
    uint8_t Status;
	M1_PCDData.Cmd				=0x0C;
	M1_PCDData.WriteToM1[0]		=0xC2;
	M1_PCDData.WriteToM1[1]		=SrcAddr;
	M1_PCDData.CRCByte			=2;
	M1_PCDData.WriteByte		=4;
    M1_PCDData.ReadBit			=0;
	CV520_CreateCRC(&M1_PCDData.WriteToM1[2]); 
    Status	=CV520_PcdComCV520();
    if((Status != 0) || (M1_PCDData.ReadBit != 4) 
	|| ((M1_PCDData.ReadFromM1[0] & 0x0F) != 0x0A))	Status = 2;             
    if (Status == 0)
    {
		M1_PCDData.Cmd			=0x0C;
        M1_PCDData.WriteToM1[0] = 0;
        M1_PCDData.WriteToM1[1] = 0;
        M1_PCDData.WriteToM1[2] = 0;
        M1_PCDData.WriteToM1[3] = 0;
		M1_PCDData.CRCByte		= 4;
		M1_PCDData.WriteByte	= 6;
		M1_PCDData.ReadBit		= 0;
        CV520_CreateCRC(&M1_PCDData.WriteToM1[4]);		
        Status=CV520_PcdComCV520();
        if (Status != 2)	Status=0;
    }    
    if (Status != 0)		Status=2;
    
    M1_PCDData.Cmd				=0x0C;
	M1_PCDData.WriteToM1[0]		=0xC2;
	M1_PCDData.WriteToM1[1]		=DstAddr;
	M1_PCDData.CRCByte			=2;
	M1_PCDData.WriteByte		=4;
    M1_PCDData.ReadBit			=0;
    CV520_CreateCRC(&M1_PCDData.WriteToM1[2]);	
    Status=CV520_PcdComCV520();
    if((Status != 0) || (M1_PCDData.ReadBit != 4) 
	|| ((M1_PCDData.ReadFromM1[0] & 0x0F) != 0x0A))	Status = 2;             
    return Status;
}









 
uint8_t CV520_PcdRead(uint8_t Addr,uint8_t *pData)
{
    uint8_t Status,i;
	M1_PCDData.Cmd				=0x0C;
	M1_PCDData.WriteToM1[0]		=0x30;
	M1_PCDData.WriteToM1[1]		=Addr;
	M1_PCDData.CRCByte			=2;
	M1_PCDData.WriteByte		=4;
    M1_PCDData.ReadBit			=0;
    CV520_CreateCRC(&M1_PCDData.WriteToM1[2]);   	
    Status=CV520_PcdComCV520();
    if ((Status == 0) && (M1_PCDData.ReadBit == 0x90))
    {
        for (i=0; i<16; i++) *pData++ =M1_PCDData.ReadFromM1[i];
    }
    else   Status	=2;   
    return Status;
}







 
uint8_t CV520_PcdHalt(void)
{
    uint8_t Status;
	M1_PCDData.Cmd				=0x0C;
	M1_PCDData.WriteToM1[0]		=0x50;
	M1_PCDData.WriteToM1[1]		=0;
	M1_PCDData.CRCByte			=2;
	M1_PCDData.WriteByte		=4;
    M1_PCDData.ReadBit			=0;
	CV520_CreateCRC(&M1_PCDData.WriteToM1[2]);	
 	Status=CV520_PcdComCV520();	
    return Status;
}







 
void	CV520_FormatData2Buff( int_fast32_t Data , uint8_t BKAddr , uint8_t *Buff)
{







 
		uint8_t		*pData;
 		*(Buff+0)	=*(Buff+2) = ~BKAddr;
		*(Buff+1)	=*(Buff+3) =  BKAddr;
		pData		=(uint8_t *)&Data;
		*(Buff+4)	=*(Buff+12)= *(pData + 0);
		*(Buff+5)	=*(Buff+13)= *(pData + 1);
		*(Buff+6)	=*(Buff+14)= *(pData + 2);
		*(Buff+7)	=*(Buff+15)= *(pData + 3);
		*(Buff+8)	= ~(*(pData + 0));
		*(Buff+9)	= ~(*(pData + 1));
		*(Buff+10)	= ~(*(pData + 2));
		*(Buff+11)	= ~(*(pData + 3));
		Data=0;
				
}







 
uint8_t	CV520_FormatBuff2Data(int_fast32_t *Data , uint8_t *Addr)
{
		uint8_t		Temp;
 		int_fast32_t	*pBuff;
		Temp	=M1_PCDData.WriteToM1[0] | M1_PCDData.WriteToM1[1];		
		if(  ( M1_PCDData.WriteToM1[1]	== M1_PCDData.WriteToM1[3])
		   &&( M1_PCDData.WriteToM1[0]	== M1_PCDData.WriteToM1[2])
		   &&( Temp == 0xff))
		 {
		   	 	*Addr	=M1_PCDData.WriteToM1[1];
		 }
		else	return  2;
		pBuff		=(int_fast32_t *)( & M1_PCDData.WriteToM1[4]);
		if(( *pBuff == (*pBuff+2)) && ((*pBuff | (*pBuff+1))==0xFFFFFFFF))
		{
			  *Data	 =*pBuff;
			  return  0;
		}
			  return 2;
}							  

























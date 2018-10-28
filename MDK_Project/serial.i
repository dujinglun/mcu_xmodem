#line 1 "..\\Common\\src\\Serial.c"
 
 
 
 
 
 
 
 
 

#line 1 "..\\Common\\inc\\LPC11Uxx.h"

 














 





 



 










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



 












 
#line 100 "..\\Common\\inc\\LPC11Uxx.h"
#line 1 "..\\Common\\inc\\system_LPC11Uxx.h"
 





















 









#line 34 "..\\Common\\inc\\system_LPC11Uxx.h"

extern uint32_t SystemCoreClock;      










 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 101 "..\\Common\\inc\\LPC11Uxx.h"



 









 

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







#line 632 "..\\Common\\inc\\LPC11Uxx.h"






#line 657 "..\\Common\\inc\\LPC11Uxx.h"


   
   
   






#line 12 "..\\Common\\src\\Serial.c"
#line 1 "..\\Common\\inc\\uart.h"


















 

















#line 45 "..\\Common\\inc\\uart.h"



 
#line 55 "..\\Common\\inc\\uart.h"

void ModemInit( void );
void UARTInit(uint32_t Baudrate);
void UART_IRQHandler(void);
void UARTSend(uint8_t *BufferPtr, uint32_t Length);
void print_string( uint8_t *str_ptr );
uint8_t get_key( void );




 
#line 13 "..\\Common\\src\\Serial.c"



 
int sendchar (int ch)  {                  

  if (ch == '\n')  {
    while (!(((LPC_USART_Type *) (0x40008000))->LSR & 0x20));
    ((LPC_USART_Type *) (0x40008000))->THR = 0x0D;                           
  }
  while (!(((LPC_USART_Type *) (0x40008000))->LSR & 0x20));
  return (((LPC_USART_Type *) (0x40008000))->THR = ch);
}


int getkey (void)  {                      

  while (!(((LPC_USART_Type *) (0x40008000))->LSR & 0x01));
  return (((LPC_USART_Type *) (0x40008000))->RBR);
}

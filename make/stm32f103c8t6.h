#ifndef STM32F103C8T6_H
#define STM32F103C8T6_H

#define __I volatile const /*!< Defines 'read only' permissions */
#define __O volatile       /*!< Defines 'write only' permissions */
#define __IO volatile      /*!< Defines 'read / write' permissions */
#define __IM \
    volatile const    /*! Defines 'read only' structure member permissions */
#define __OM volatile /*! Defines 'write only' structure member permissions */
#define __IOM \
    volatile /*! Defines 'read / write' structure member permissions */

typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;


/**
  \brief  Union type to access the Application Program Status Register (APSR).
 */
typedef union {
    struct {
        uint32_t _reserved0 : 27; /*!< bit:  0..26  Reserved */
        uint32_t Q : 1;           /*!< bit:     27  Saturation condition flag */
        uint32_t V : 1; /*!< bit:     28  Overflow condition code flag */
        uint32_t C : 1; /*!< bit:     29  Carry condition code flag */
        uint32_t Z : 1; /*!< bit:     30  Zero condition code flag */
        uint32_t N : 1; /*!< bit:     31  Negative condition code flag */
    } b;                /*!< Structure used for bit  access */
    uint32_t w;         /*!< Type      used for word access */
} APSR_Type;

/**
  \brief  Union type to access the Interrupt Program Status Register (IPSR).
 */
typedef union {
    struct {
        uint32_t ISR : 9;         /*!< bit:  0.. 8  Exception number */
        uint32_t _reserved0 : 23; /*!< bit:  9..31  Reserved */
    } b;                          /*!< Structure used for bit  access */
    uint32_t w;                   /*!< Type      used for word access */
} IPSR_Type;

/**
  \brief  Union type to access the Special-Purpose Program Status Registers
  (xPSR).
 */
typedef union {
    struct {
        uint32_t ISR : 9;         /*!< bit:  0.. 8  Exception number */
        uint32_t _reserved0 : 15; /*!< bit:  9..23  Reserved */
        uint32_t T : 1;           /*!< bit:     24  Thumb bit        (read 0) */
        uint32_t IT : 2;          /*!< bit: 25..26  saved IT state   (read 0) */
        uint32_t Q : 1;           /*!< bit:     27  Saturation condition flag */
        uint32_t V : 1; /*!< bit:     28  Overflow condition code flag */
        uint32_t C : 1; /*!< bit:     29  Carry condition code flag */
        uint32_t Z : 1; /*!< bit:     30  Zero condition code flag */
        uint32_t N : 1; /*!< bit:     31  Negative condition code flag */
    } b;                /*!< Structure used for bit  access */
    uint32_t w;         /*!< Type      used for word access */
} xPSR_Type;

/**
  \brief  Union type to access the Control Registers (CONTROL).
 */
typedef union {
    struct {
        uint32_t
            nPRIV : 1; /*!< bit:      0  Execution privilege in Thread mode */
        uint32_t SPSEL : 1;       /*!< bit:      1  Stack to be used */
        uint32_t _reserved1 : 30; /*!< bit:  2..31  Reserved */
    } b;                          /*!< Structure used for bit  access */
    uint32_t w;                   /*!< Type      used for word access */
} CONTROL_Type;

typedef struct {
    __IOM uint32_t
        ISER[8U]; /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
    uint32_t RESERVED0[24U];
    __IOM uint32_t
        ICER[8U]; /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
    uint32_t RSERVED1[24U];
    __IOM uint32_t
        ISPR[8U]; /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
    uint32_t RESERVED2[24U];
    __IOM uint32_t
        ICPR[8U]; /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
    uint32_t RESERVED3[24U];
    __IOM uint32_t
        IABR[8U]; /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
    uint32_t RESERVED4[56U];
    __IOM uint8_t
        IP[240U]; /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit
                     wide) */
    uint32_t RESERVED5[644U];
    __OM uint32_t
        STIR; /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Type;

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct {
    __IM uint32_t CPUID; /*!< Offset: 0x000 (R/ )  CPUID Base Register */
    __IOM uint32_t
        ICSR; /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
    __IOM uint32_t
        VTOR; /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
    __IOM uint32_t AIRCR; /*!< Offset: 0x00C (R/W)  Application Interrupt and
                             Reset Control Register */
    __IOM uint32_t SCR;   /*!< Offset: 0x010 (R/W)  System Control Register */
    __IOM uint32_t
        CCR; /*!< Offset: 0x014 (R/W)  Configuration Control Register */
    __IOM uint8_t SHP[12U]; /*!< Offset: 0x018 (R/W)  System Handlers Priority
                               Registers (4-7, 8-11, 12-15) */
    __IOM uint32_t SHCSR;   /*!< Offset: 0x024 (R/W)  System Handler Control and
                               State Register */
    __IOM uint32_t
        CFSR; /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
    __IOM uint32_t HFSR; /*!< Offset: 0x02C (R/W)  HardFault Status Register */
    __IOM uint32_t
        DFSR; /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
    __IOM uint32_t
        MMFAR; /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
    __IOM uint32_t BFAR; /*!< Offset: 0x038 (R/W)  BusFault Address Register */
    __IOM uint32_t
        AFSR; /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
    __IM uint32_t
        PFR[2U];       /*!< Offset: 0x040 (R/ )  Processor Feature Register */
    __IM uint32_t DFR; /*!< Offset: 0x048 (R/ )  Debug Feature Register */
    __IM uint32_t ADR; /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
    __IM uint32_t
        MMFR[4U]; /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
    __IM uint32_t ISAR
        [5U]; /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
    uint32_t RESERVED0[5U];
    __IOM uint32_t
        CPACR; /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct {
    __IOM uint32_t
        CTRL; /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
    __IOM uint32_t
        LOAD; /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
    __IOM uint32_t
        VAL; /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
    __IM uint32_t
        CALIB; /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

/**
  \brief  Structure type to access the Instrumentation Trace Macrocell Register
  (ITM).
 */
typedef struct {
    __OM union {
        __OM uint8_t u8;   /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit */
        __OM uint16_t u16; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit */
        __OM uint32_t u32; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit */
    } PORT[32U]; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port Registers */
    uint32_t RESERVED0[864U];
    __IOM uint32_t TER; /*!< Offset: 0xE00 (R/W)  ITM Trace Enable Register */
    uint32_t RESERVED1[15U];
    __IOM uint32_t
        TPR; /*!< Offset: 0xE40 (R/W)  ITM Trace Privilege Register */
    uint32_t RESERVED2[15U];
    __IOM uint32_t TCR; /*!< Offset: 0xE80 (R/W)  ITM Trace Control Register */
    uint32_t RESERVED3[29U];
    __OM uint32_t
        IWR; /*!< Offset: 0xEF8 ( /W)  ITM Integration Write Register */
    __IM uint32_t
        IRR; /*!< Offset: 0xEFC (R/ )  ITM Integration Read Register */
    __IOM uint32_t
        IMCR; /*!< Offset: 0xF00 (R/W)  ITM Integration Mode Control Register */
    uint32_t RESERVED4[43U];
    __OM uint32_t LAR; /*!< Offset: 0xFB0 ( /W)  ITM Lock Access Register */
    __IM uint32_t LSR; /*!< Offset: 0xFB4 (R/ )  ITM Lock Status Register */
    uint32_t RESERVED5[6U];
    __IM uint32_t PID4; /*!< Offset: 0xFD0 (R/ )  ITM Peripheral Identification
                           Register #4 */
    __IM uint32_t PID5; /*!< Offset: 0xFD4 (R/ )  ITM Peripheral Identification
                           Register #5 */
    __IM uint32_t PID6; /*!< Offset: 0xFD8 (R/ )  ITM Peripheral Identification
                           Register #6 */
    __IM uint32_t PID7; /*!< Offset: 0xFDC (R/ )  ITM Peripheral Identification
                           Register #7 */
    __IM uint32_t PID0; /*!< Offset: 0xFE0 (R/ )  ITM Peripheral Identification
                           Register #0 */
    __IM uint32_t PID1; /*!< Offset: 0xFE4 (R/ )  ITM Peripheral Identification
                           Register #1 */
    __IM uint32_t PID2; /*!< Offset: 0xFE8 (R/ )  ITM Peripheral Identification
                           Register #2 */
    __IM uint32_t PID3; /*!< Offset: 0xFEC (R/ )  ITM Peripheral Identification
                           Register #3 */
    __IM uint32_t CID0; /*!< Offset: 0xFF0 (R/ )  ITM Component  Identification
                           Register #0 */
    __IM uint32_t CID1; /*!< Offset: 0xFF4 (R/ )  ITM Component  Identification
                           Register #1 */
    __IM uint32_t CID2; /*!< Offset: 0xFF8 (R/ )  ITM Component  Identification
                           Register #2 */
    __IM uint32_t CID3; /*!< Offset: 0xFFC (R/ )  ITM Component  Identification
                           Register #3 */
} ITM_Type;

/**
  \brief  Structure type to access the Data Watchpoint and Trace Register (DWT).
 */
typedef struct {
    __IOM uint32_t CTRL;   /*!< Offset: 0x000 (R/W)  Control Register */
    __IOM uint32_t CYCCNT; /*!< Offset: 0x004 (R/W)  Cycle Count Register */
    __IOM uint32_t CPICNT; /*!< Offset: 0x008 (R/W)  CPI Count Register */
    __IOM uint32_t
        EXCCNT; /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
    __IOM uint32_t SLEEPCNT; /*!< Offset: 0x010 (R/W)  Sleep Count Register */
    __IOM uint32_t LSUCNT;   /*!< Offset: 0x014 (R/W)  LSU Count Register */
    __IOM uint32_t
        FOLDCNT; /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
    __IM uint32_t
        PCSR; /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
    __IOM uint32_t COMP0;     /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
    __IOM uint32_t MASK0;     /*!< Offset: 0x024 (R/W)  Mask Register 0 */
    __IOM uint32_t FUNCTION0; /*!< Offset: 0x028 (R/W)  Function Register 0 */
    uint32_t RESERVED0[1U];
    __IOM uint32_t COMP1;     /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
    __IOM uint32_t MASK1;     /*!< Offset: 0x034 (R/W)  Mask Register 1 */
    __IOM uint32_t FUNCTION1; /*!< Offset: 0x038 (R/W)  Function Register 1 */
    uint32_t RESERVED1[1U];
    __IOM uint32_t COMP2;     /*!< Offset: 0x040 (R/W)  Comparator Register 2 */
    __IOM uint32_t MASK2;     /*!< Offset: 0x044 (R/W)  Mask Register 2 */
    __IOM uint32_t FUNCTION2; /*!< Offset: 0x048 (R/W)  Function Register 2 */
    uint32_t RESERVED2[1U];
    __IOM uint32_t COMP3;     /*!< Offset: 0x050 (R/W)  Comparator Register 3 */
    __IOM uint32_t MASK3;     /*!< Offset: 0x054 (R/W)  Mask Register 3 */
    __IOM uint32_t FUNCTION3; /*!< Offset: 0x058 (R/W)  Function Register 3 */
} DWT_Type;

/**
  \brief  Structure type to access the Trace Port Interface Register (TPI).
 */
typedef struct {
    __IOM uint32_t
        SSPSR; /*!< Offset: 0x000 (R/ )  Supported Parallel Port Size Register
                */
    __IOM uint32_t
        CSPSR; /*!< Offset: 0x004 (R/W)  Current Parallel Port Size Register */
    uint32_t RESERVED0[2U];
    __IOM uint32_t
        ACPR; /*!< Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register */
    uint32_t RESERVED1[55U];
    __IOM uint32_t
        SPPR; /*!< Offset: 0x0F0 (R/W)  Selected Pin Protocol Register */
    uint32_t RESERVED2[131U];
    __IM uint32_t
        FFSR; /*!< Offset: 0x300 (R/ )  Formatter and Flush Status Register */
    __IOM uint32_t
        FFCR; /*!< Offset: 0x304 (R/W)  Formatter and Flush Control Register */
    __IM uint32_t FSCR; /*!< Offset: 0x308 (R/ )  Formatter Synchronization
                           Counter Register */
    uint32_t RESERVED3[759U];
    __IM uint32_t TRIGGER;   /*!< Offset: 0xEE8 (R/ )  TRIGGER */
    __IM uint32_t FIFO0;     /*!< Offset: 0xEEC (R/ )  Integration ETM Data */
    __IM uint32_t ITATBCTR2; /*!< Offset: 0xEF0 (R/ )  ITATBCTR2 */
    uint32_t RESERVED4[1U];
    __IM uint32_t ITATBCTR0; /*!< Offset: 0xEF8 (R/ )  ITATBCTR0 */
    __IM uint32_t FIFO1;     /*!< Offset: 0xEFC (R/ )  Integration ITM Data */
    __IOM uint32_t ITCTRL; /*!< Offset: 0xF00 (R/W)  Integration Mode Control */
    uint32_t RESERVED5[39U];
    __IOM uint32_t CLAIMSET; /*!< Offset: 0xFA0 (R/W)  Claim tag set */
    __IOM uint32_t CLAIMCLR; /*!< Offset: 0xFA4 (R/W)  Claim tag clear */
    uint32_t RESERVED7[8U];
    __IM uint32_t DEVID;   /*!< Offset: 0xFC8 (R/ )  TPIU_DEVID */
    __IM uint32_t DEVTYPE; /*!< Offset: 0xFCC (R/ )  TPIU_DEVTYPE */
} TPI_Type;

/**
  \brief  Structure type to access the Memory Protection Unit (MPU).
 */
typedef struct {
    __IM uint32_t TYPE;  /*!< Offset: 0x000 (R/ )  MPU Type Register */
    __IOM uint32_t CTRL; /*!< Offset: 0x004 (R/W)  MPU Control Register */
    __IOM uint32_t RNR;  /*!< Offset: 0x008 (R/W)  MPU Region RNRber Register */
    __IOM uint32_t
        RBAR; /*!< Offset: 0x00C (R/W)  MPU Region Base Address Register */
    __IOM uint32_t
        RASR; /*!< Offset: 0x010 (R/W)  MPU Region Attribute and Size Register
               */
    __IOM uint32_t RBAR_A1; /*!< Offset: 0x014 (R/W)  MPU Alias 1 Region Base
                               Address Register */
    __IOM uint32_t
        RASR_A1; /*!< Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size
         Register */
    __IOM uint32_t RBAR_A2; /*!< Offset: 0x01C (R/W)  MPU Alias 2 Region Base
                               Address Register */
    __IOM uint32_t
        RASR_A2; /*!< Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size
         Register */
    __IOM uint32_t RBAR_A3; /*!< Offset: 0x024 (R/W)  MPU Alias 3 Region Base
                               Address Register */
    __IOM uint32_t
        RASR_A3; /*!< Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size
                    Register */
} MPU_Type;

/**
  \brief  Structure type to access the Core Debug Register (CoreDebug).
 */
typedef struct {
    __IOM uint32_t DHCSR; /*!< Offset: 0x000 (R/W)  Debug Halting Control and
                             Status Register */
    __OM uint32_t
        DCRSR; /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register
                */
    __IOM uint32_t
        DCRDR; /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register */
    __IOM uint32_t DEMCR; /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor
                             Control Register */
} CoreDebug_Type;

/* Memory mapping of Cortex-M3 Hardware */
#define SCS_BASE (0xE000E000UL)       /*!< System Control Space Base Address */
#define ITM_BASE (0xE0000000UL)       /*!< ITM Base Address */
#define DWT_BASE (0xE0001000UL)       /*!< DWT Base Address */
#define TPI_BASE (0xE0040000UL)       /*!< TPI Base Address */
#define CoreDebug_BASE (0xE000EDF0UL) /*!< Core Debug Base Address */
#define SysTick_BASE (SCS_BASE + 0x0010UL) /*!< SysTick Base Address */
#define NVIC_BASE (SCS_BASE + 0x0100UL)    /*!< NVIC Base Address */
#define SCB_BASE                                                               \
    (SCS_BASE + 0x0D00UL) /*!< System Control Block Base Address \ \ \ \ \ \ \ \
                           * \ \ \ \ \ \ \                                     \
                           * \ \ \ \ \ \                                       \
                           * \ \ \ \ \ \                                       \
                           * \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \               \
                           */

#define SCnSCB \
    ((SCnSCB_Type *)SCS_BASE)      /*!< System control Register not in SCB */
#define SCB ((SCB_Type *)SCB_BASE) /*!< SCB configuration struct */
#define SysTick \
    ((SysTick_Type *)SysTick_BASE)    /*!< SysTick configuration struct */
#define NVIC ((NVIC_Type *)NVIC_BASE) /*!< NVIC configuration struct */
#define ITM ((ITM_Type *)ITM_BASE)    /*!< ITM configuration struct */
#define DWT ((DWT_Type *)DWT_BASE)    /*!< DWT configuration struct */
#define TPI ((TPI_Type *)TPI_BASE)    /*!< TPI configuration struct */
#define CoreDebug \
    ((CoreDebug_Type *)CoreDebug_BASE) /*!< Core Debug configuration struct */

/**
 * @brief STM32F10x Interrupt Number Definition, according to the selected
 * device
 *        in @ref Library_configuration_section
 */

/*!< Interrupt Number Definition */
typedef enum {
    /******  Cortex-M3 Processor Exceptions Numbers
     ***************************************************/
    NonMaskableInt_IRQn = -14,   /*!< 2 Non Maskable Interrupt */
    HardFault_IRQn = -13,        /*!< 3 Cortex-M3 Hard Fault Interrupt */
    MemoryManagement_IRQn = -12, /*!< 4 Cortex-M3 Memory Management Interrupt */
    BusFault_IRQn = -11,         /*!< 5 Cortex-M3 Bus Fault Interrupt */
    UsageFault_IRQn = -10,       /*!< 6 Cortex-M3 Usage Fault Interrupt */
    SVCall_IRQn = -5,       /*!< 11 Cortex-M3 SV Call Interrupt            */
    DebugMonitor_IRQn = -4, /*!< 12 Cortex-M3 Debug Monitor Interrupt */
    PendSV_IRQn = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt            */
    SysTick_IRQn = -1,      /*!< 15 Cortex-M3 System Tick Interrupt */

    /******  STM32 specific Interrupt Numbers
     *********************************************************/
    WWDG_IRQn = 0, /*!< Window WatchDog Interrupt                            */
    PVD_IRQn = 1,  /*!< PVD through EXTI Line detection Interrupt            */
    TAMPER_IRQn = 2, /*!< Tamper Interrupt */
    RTC_IRQn = 3,   /*!< RTC global Interrupt                                 */
    FLASH_IRQn = 4, /*!< FLASH global Interrupt                               */
    RCC_IRQn = 5,   /*!< RCC global Interrupt                                 */
    EXTI0_IRQn = 6, /*!< EXTI Line0 Interrupt                                 */
    EXTI1_IRQn = 7, /*!< EXTI Line1 Interrupt                                 */
    EXTI2_IRQn = 8, /*!< EXTI Line2 Interrupt                                 */
    EXTI3_IRQn = 9, /*!< EXTI Line3 Interrupt                                 */
    EXTI4_IRQn = 10,         /*!< EXTI Line4 Interrupt         */
    DMA1_Channel1_IRQn = 11, /*!< DMA1 Channel 1 global Interrupt */
    DMA1_Channel2_IRQn = 12, /*!< DMA1 Channel 2 global Interrupt */
    DMA1_Channel3_IRQn = 13, /*!< DMA1 Channel 3 global Interrupt */
    DMA1_Channel4_IRQn = 14, /*!< DMA1 Channel 4 global Interrupt */
    DMA1_Channel5_IRQn = 15, /*!< DMA1 Channel 5 global Interrupt */
    DMA1_Channel6_IRQn = 16, /*!< DMA1 Channel 6 global Interrupt */
    DMA1_Channel7_IRQn = 17, /*!< DMA1 Channel 7 global Interrupt */
    ADC1_2_IRQn = 18,        /*!< ADC1 and ADC2 global Interrupt        */
    USB_HP_CAN1_TX_IRQn =
        19, /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn =
        20, /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn = 21,     /*!< CAN1 RX1 Interrupt */
    CAN1_SCE_IRQn = 22,     /*!< CAN1 SCE Interrupt */
    EXTI9_5_IRQn = 23,      /*!< External Line[9:5] Interrupts */
    TIM1_BRK_IRQn = 24,     /*!< TIM1 Break Interrupt */
    TIM1_UP_IRQn = 25,      /*!< TIM1 Update Interrupt */
    TIM1_TRG_COM_IRQn = 26, /*!< TIM1 Trigger and Commutation Interrupt */
    TIM1_CC_IRQn = 27,      /*!< TIM1 Capture Compare Interrupt */
    TIM2_IRQn = 28, /*!< TIM2 global Interrupt                                */
    TIM3_IRQn = 29, /*!< TIM3 global Interrupt                                */
    TIM4_IRQn = 30, /*!< TIM4 global Interrupt                                */
    I2C1_EV_IRQn = 31, /*!< I2C1 Event Interrupt */
    I2C1_ER_IRQn = 32, /*!< I2C1 Error Interrupt */
    I2C2_EV_IRQn = 33, /*!< I2C2 Event Interrupt */
    I2C2_ER_IRQn = 34, /*!< I2C2 Error Interrupt */
    SPI1_IRQn = 35, /*!< SPI1 global Interrupt                                */
    SPI2_IRQn = 36, /*!< SPI2 global Interrupt                                */
    USART1_IRQn = 37,    /*!< USART1 global Interrupt    */
    USART2_IRQn = 38,    /*!< USART2 global Interrupt    */
    USART3_IRQn = 39,    /*!< USART3 global Interrupt    */
    EXTI15_10_IRQn = 40, /*!< External Line[15:10] Interrupts */
    RTC_Alarm_IRQn = 41, /*!< RTC Alarm through EXTI Line Interrupt */
    USBWakeUp_IRQn =
        42, /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
} IRQn_Type;

/**
 * @brief Analog to Digital Converter
 */

typedef struct {
    __IO uint32_t SR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMPR1;
    __IO uint32_t SMPR2;
    __IO uint32_t JOFR1;
    __IO uint32_t JOFR2;
    __IO uint32_t JOFR3;
    __IO uint32_t JOFR4;
    __IO uint32_t HTR;
    __IO uint32_t LTR;
    __IO uint32_t SQR1;
    __IO uint32_t SQR2;
    __IO uint32_t SQR3;
    __IO uint32_t JSQR;
    __IO uint32_t JDR1;
    __IO uint32_t JDR2;
    __IO uint32_t JDR3;
    __IO uint32_t JDR4;
    __IO uint32_t DR;
} ADC_TypeDef;

typedef struct {
    __IO uint32_t SR;  /*!< ADC status register,    used for ADC multimode (bits
                          common to several ADC instances). Address offset: ADC1
                          base address         */
    __IO uint32_t CR1; /*!< ADC control register 1, used for ADC multimode (bits
                          common to several ADC instances). Address offset: ADC1
                          base address + 0x04  */
    __IO uint32_t CR2; /*!< ADC control register 2, used for ADC multimode (bits
                          common to several ADC instances). Address offset: ADC1
                          base address + 0x08  */
    uint32_t RESERVED[16];
    __IO uint32_t DR; /*!< ADC data register,      used for ADC multimode (bits
                         common to several ADC instances). Address offset: ADC1
                         base address + 0x4C  */
} ADC_Common_TypeDef;

/**
 * @brief Backup Registers
 */

typedef struct {
    uint32_t RESERVED0;
    __IO uint32_t DR1;
    __IO uint32_t DR2;
    __IO uint32_t DR3;
    __IO uint32_t DR4;
    __IO uint32_t DR5;
    __IO uint32_t DR6;
    __IO uint32_t DR7;
    __IO uint32_t DR8;
    __IO uint32_t DR9;
    __IO uint32_t DR10;
    __IO uint32_t RTCCR;
    __IO uint32_t CR;
    __IO uint32_t CSR;
} BKP_TypeDef;

/**
 * @brief Controller Area Network TxMailBox
 */

typedef struct {
    __IO uint32_t TIR;
    __IO uint32_t TDTR;
    __IO uint32_t TDLR;
    __IO uint32_t TDHR;
} CAN_TxMailBox_TypeDef;

/**
 * @brief Controller Area Network FIFOMailBox
 */

typedef struct {
    __IO uint32_t RIR;
    __IO uint32_t RDTR;
    __IO uint32_t RDLR;
    __IO uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;

/**
 * @brief Controller Area Network FilterRegister
 */

typedef struct {
    __IO uint32_t FR1;
    __IO uint32_t FR2;
} CAN_FilterRegister_TypeDef;

/**
 * @brief Controller Area Network
 */

typedef struct {
    __IO uint32_t MCR;
    __IO uint32_t MSR;
    __IO uint32_t TSR;
    __IO uint32_t RF0R;
    __IO uint32_t RF1R;
    __IO uint32_t IER;
    __IO uint32_t ESR;
    __IO uint32_t BTR;
    uint32_t RESERVED0[88];
    CAN_TxMailBox_TypeDef sTxMailBox[3];
    CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
    uint32_t RESERVED1[12];
    __IO uint32_t FMR;
    __IO uint32_t FM1R;
    uint32_t RESERVED2;
    __IO uint32_t FS1R;
    uint32_t RESERVED3;
    __IO uint32_t FFA1R;
    uint32_t RESERVED4;
    __IO uint32_t FA1R;
    uint32_t RESERVED5[8];
    CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

/**
 * @brief CRC calculation unit
 */

typedef struct {
    __IO uint32_t DR;  /*!< CRC Data register,                           Address
                          offset: 0x00 */
    __IO uint8_t IDR;  /*!< CRC Independent data register,               Address
                          offset: 0x04 */
    uint8_t RESERVED0; /*!< Reserved,                                    Address
                          offset: 0x05 */
    uint16_t RESERVED1; /*!< Reserved, Address offset: 0x06 */
    __IO uint32_t CR; /*!< CRC Control register,                        Address
                         offset: 0x08 */
} CRC_TypeDef;

/**
 * @brief Debug MCU
 */

typedef struct {
    __IO uint32_t IDCODE;
    __IO uint32_t CR;
} DBGMCU_TypeDef;

/**
 * @brief DMA Controller
 */

typedef struct {
    __IO uint32_t CCR;
    __IO uint32_t CNDTR;
    __IO uint32_t CPAR;
    __IO uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct {
    __IO uint32_t ISR;
    __IO uint32_t IFCR;
} DMA_TypeDef;

/**
 * @brief External Interrupt/Event Controller
 */

typedef struct {
    __IO uint32_t IMR;
    __IO uint32_t EMR;
    __IO uint32_t RTSR;
    __IO uint32_t FTSR;
    __IO uint32_t SWIER;
    __IO uint32_t PR;
} EXTI_TypeDef;

/**
 * @brief FLASH Registers
 */

typedef struct {
    __IO uint32_t ACR;
    __IO uint32_t KEYR;
    __IO uint32_t OPTKEYR;
    __IO uint32_t SR;
    __IO uint32_t CR;
    __IO uint32_t AR;
    __IO uint32_t RESERVED;
    __IO uint32_t OBR;
    __IO uint32_t WRPR;
} FLASH_TypeDef;

/**
 * @brief Option Bytes Registers
 */

typedef struct {
    __IO uint16_t RDP;
    __IO uint16_t USER;
    __IO uint16_t Data0;
    __IO uint16_t Data1;
    __IO uint16_t WRP0;
    __IO uint16_t WRP1;
    __IO uint16_t WRP2;
    __IO uint16_t WRP3;
} OB_TypeDef;

/**
 * @brief General Purpose I/O
 */

typedef struct {
    __IO uint32_t CRL;
    __IO uint32_t CRH;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t BRR;
    __IO uint32_t LCKR;
} GPIO_TypeDef;

/**
 * @brief Alternate Function I/O
 */

typedef struct {
    __IO uint32_t EVCR;
    __IO uint32_t MAPR;
    __IO uint32_t EXTICR[4];
    uint32_t RESERVED0;
    __IO uint32_t MAPR2;
} AFIO_TypeDef;
/**
 * @brief Inter Integrated Circuit Interface
 */

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t OAR1;
    __IO uint32_t OAR2;
    __IO uint32_t DR;
    __IO uint32_t SR1;
    __IO uint32_t SR2;
    __IO uint32_t CCR;
    __IO uint32_t TRISE;
} I2C_TypeDef;

/**
 * @brief Independent WATCHDOG
 */

typedef struct {
    __IO uint32_t KR;  /*!< Key register,                                Address
                          offset: 0x00 */
    __IO uint32_t PR;  /*!< Prescaler register,                          Address
                          offset: 0x04 */
    __IO uint32_t RLR; /*!< Reload register,                             Address
                          offset: 0x08 */
    __IO uint32_t SR;  /*!< Status register,                             Address
                          offset: 0x0C */
} IWDG_TypeDef;

/**
 * @brief Power Control
 */

typedef struct {
    __IO uint32_t CR;
    __IO uint32_t CSR;
} PWR_TypeDef;

/**
 * @brief Reset and Clock Control
 */

typedef struct {
    __IO uint32_t CR;
    __IO uint32_t CFGR;
    __IO uint32_t CIR;
    __IO uint32_t APB2RSTR;
    __IO uint32_t APB1RSTR;
    __IO uint32_t AHBENR;
    __IO uint32_t APB2ENR;
    __IO uint32_t APB1ENR;
    __IO uint32_t BDCR;
    __IO uint32_t CSR;

} RCC_TypeDef;

/**
 * @brief Real-Time Clock
 */

typedef struct {
    __IO uint32_t CRH;
    __IO uint32_t CRL;
    __IO uint32_t PRLH;
    __IO uint32_t PRLL;
    __IO uint32_t DIVH;
    __IO uint32_t DIVL;
    __IO uint32_t CNTH;
    __IO uint32_t CNTL;
    __IO uint32_t ALRH;
    __IO uint32_t ALRL;
} RTC_TypeDef;

/**
 * @brief SD host Interface
 */

typedef struct {
    __IO uint32_t POWER;
    __IO uint32_t CLKCR;
    __IO uint32_t ARG;
    __IO uint32_t CMD;
    __I uint32_t RESPCMD;
    __I uint32_t RESP1;
    __I uint32_t RESP2;
    __I uint32_t RESP3;
    __I uint32_t RESP4;
    __IO uint32_t DTIMER;
    __IO uint32_t DLEN;
    __IO uint32_t DCTRL;
    __I uint32_t DCOUNT;
    __I uint32_t STA;
    __IO uint32_t ICR;
    __IO uint32_t MASK;
    uint32_t RESERVED0[2];
    __I uint32_t FIFOCNT;
    uint32_t RESERVED1[13];
    __IO uint32_t FIFO;
} SDIO_TypeDef;

/**
 * @brief Serial Peripheral Interface
 */

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t CRCPR;
    __IO uint32_t RXCRCR;
    __IO uint32_t TXCRCR;
    __IO uint32_t I2SCFGR;
} SPI_TypeDef;

/**
 * @brief TIM Timers
 */
typedef struct {
    __IO uint32_t CR1; /*!< TIM control register 1,                      Address
                          offset: 0x00 */
    __IO uint32_t CR2; /*!< TIM control register 2,                      Address
                          offset: 0x04 */
    __IO uint32_t SMCR; /*!< TIM slave Mode Control register, Address offset:
                           0x08 */
    __IO uint32_t DIER; /*!< TIM DMA/interrupt enable register, Address offset:
                  0x0C */
    __IO uint32_t SR;  /*!< TIM status register,                         Address
                          offset: 0x10 */
    __IO uint32_t EGR; /*!< TIM event generation register,               Address
                          offset: 0x14 */
    __IO uint32_t
        CCMR1; /*!< TIM  capture/compare mode register 1,        Address offset:
                  0x18 */
    __IO uint32_t
        CCMR2; /*!< TIM  capture/compare mode register 2,        Address offset:
                  0x1C */
    __IO uint32_t
        CCER; /*!< TIM capture/compare enable register,         Address offset:
        0x20 */
    __IO uint32_t CNT; /*!< TIM counter register,                        Address
                          offset: 0x24 */
    __IO uint32_t PSC; /*!< TIM prescaler register,                      Address
                          offset: 0x28 */
    __IO uint32_t ARR; /*!< TIM auto-reload register,                    Address
                          offset: 0x2C */
    __IO uint32_t RCR; /*!< TIM  repetition counter register,            Address
                          offset: 0x30 */
    __IO uint32_t CCR1; /*!< TIM capture/compare register 1, Address offset:
                           0x34 */
    __IO uint32_t CCR2; /*!< TIM capture/compare register 2, Address offset:
                           0x38 */
    __IO uint32_t CCR3; /*!< TIM capture/compare register 3, Address offset:
                           0x3C */
    __IO uint32_t CCR4; /*!< TIM capture/compare register 4, Address offset:
                           0x40 */
    __IO uint32_t BDTR; /*!< TIM break and dead-time register, Address offset:
                  0x44 */
    __IO uint32_t DCR; /*!< TIM DMA control register,                    Address
                          offset: 0x48 */
    __IO uint32_t
        DMAR; /*!< TIM DMA address for full transfer register,  Address offset:
         0x4C */
    __IO uint32_t OR; /*!< TIM option register,                         Address
                         offset: 0x50 */
} TIM_TypeDef;

/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */

typedef struct {
    __IO uint32_t SR;   /*!< USART Status register,                   Address
                           offset: 0x00 */
    __IO uint32_t DR;   /*!< USART Data register,                     Address
                           offset: 0x04 */
    __IO uint32_t BRR;  /*!< USART Baud rate register,                Address
                           offset: 0x08 */
    __IO uint32_t CR1;  /*!< USART Control register 1,                Address
                           offset: 0x0C */
    __IO uint32_t CR2;  /*!< USART Control register 2,                Address
                           offset: 0x10 */
    __IO uint32_t CR3;  /*!< USART Control register 3,                Address
                           offset: 0x14 */
    __IO uint32_t GTPR; /*!< USART Guard time and prescaler register, Address
                           offset: 0x18 */
} USART_TypeDef;

/**
 * @brief Universal Serial Bus Full Speed Device
 */

typedef struct {
    __IO uint16_t EP0R; /*!< USB Endpoint 0 register,                   Address
                  offset: 0x00 */
    __IO uint16_t RESERVED0; /*!< Reserved */
    __IO uint16_t EP1R; /*!< USB Endpoint 1 register,                   Address
                  offset: 0x04 */
    __IO uint16_t RESERVED1; /*!< Reserved */
    __IO uint16_t EP2R; /*!< USB Endpoint 2 register,                   Address
                  offset: 0x08 */
    __IO uint16_t RESERVED2; /*!< Reserved */
    __IO uint16_t EP3R; /*!< USB Endpoint 3 register,                   Address
                  offset: 0x0C */
    __IO uint16_t RESERVED3; /*!< Reserved */
    __IO uint16_t EP4R; /*!< USB Endpoint 4 register,                   Address
                  offset: 0x10 */
    __IO uint16_t RESERVED4; /*!< Reserved */
    __IO uint16_t EP5R; /*!< USB Endpoint 5 register,                   Address
                  offset: 0x14 */
    __IO uint16_t RESERVED5; /*!< Reserved */
    __IO uint16_t EP6R; /*!< USB Endpoint 6 register,                   Address
                  offset: 0x18 */
    __IO uint16_t RESERVED6; /*!< Reserved */
    __IO uint16_t EP7R; /*!< USB Endpoint 7 register,                   Address
                  offset: 0x1C */
    __IO uint16_t RESERVED7[17]; /*!< Reserved */
    __IO uint16_t CNTR; /*!< Control register,                          Address
                  offset: 0x40 */
    __IO uint16_t RESERVED8; /*!< Reserved */
    __IO uint16_t ISTR; /*!< Interrupt status register,                 Address
                  offset: 0x44 */
    __IO uint16_t RESERVED9; /*!< Reserved */
    __IO uint16_t FNR; /*!< Frame number register,                     Address
                offset: 0x48 */
    __IO uint16_t RESERVEDA; /*!< Reserved */
    __IO uint16_t DADDR; /*!< Device address register,                   Address
                    offset: 0x4C */
    __IO uint16_t RESERVEDB; /*!< Reserved */
    __IO uint16_t BTABLE; /*!< Buffer Table address register,    Address offset:
                0x50 */
    __IO uint16_t RESERVEDC; /*!< Reserved */
} USB_TypeDef;

/**
 * @brief Window WATCHDOG
 */

typedef struct {
    __IO uint32_t CR;  /*!< WWDG Control register,       Address offset: 0x00 */
    __IO uint32_t CFR; /*!< WWDG Configuration register, Address offset: 0x04 */
    __IO uint32_t SR;  /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

#define FLASH_BASE 0x08000000U /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END 0x0801FFFFU /*!< FLASH END address of bank1 */
#define SRAM_BASE 0x20000000U /*!< SRAM base address in the alias region */
#define PERIPH_BASE \
    0x40000000U /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE \
    0x22000000U /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE \
    0x42000000U /*!< Peripheral base address in the bit-band region */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (PERIPH_BASE + 0x00010000U)
#define AHBPERIPH_BASE (PERIPH_BASE + 0x00020000U)

#define TIM2_BASE (APB1PERIPH_BASE + 0x00000000U)
#define TIM3_BASE (APB1PERIPH_BASE + 0x00000400U)
#define TIM4_BASE (APB1PERIPH_BASE + 0x00000800U)
#define RTC_BASE (APB1PERIPH_BASE + 0x00002800U)
#define WWDG_BASE (APB1PERIPH_BASE + 0x00002C00U)
#define IWDG_BASE (APB1PERIPH_BASE + 0x00003000U)
#define SPI2_BASE (APB1PERIPH_BASE + 0x00003800U)
#define USART2_BASE (APB1PERIPH_BASE + 0x00004400U)
#define USART3_BASE (APB1PERIPH_BASE + 0x00004800U)
#define I2C1_BASE (APB1PERIPH_BASE + 0x00005400U)
#define I2C2_BASE (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASE (APB1PERIPH_BASE + 0x00006400U)
#define BKP_BASE (APB1PERIPH_BASE + 0x00006C00U)
#define PWR_BASE (APB1PERIPH_BASE + 0x00007000U)
#define AFIO_BASE (APB2PERIPH_BASE + 0x00000000U)
#define EXTI_BASE (APB2PERIPH_BASE + 0x00000400U)
#define GPIOA_BASE (APB2PERIPH_BASE + 0x00000800U)
#define GPIOB_BASE (APB2PERIPH_BASE + 0x00000C00U)
#define GPIOC_BASE (APB2PERIPH_BASE + 0x00001000U)
#define GPIOD_BASE (APB2PERIPH_BASE + 0x00001400U)
#define GPIOE_BASE (APB2PERIPH_BASE + 0x00001800U)
#define ADC1_BASE (APB2PERIPH_BASE + 0x00002400U)
#define ADC2_BASE (APB2PERIPH_BASE + 0x00002800U)
#define TIM1_BASE (APB2PERIPH_BASE + 0x00002C00U)
#define SPI1_BASE (APB2PERIPH_BASE + 0x00003000U)
#define USART1_BASE (APB2PERIPH_BASE + 0x00003800U)

#define SDIO_BASE (PERIPH_BASE + 0x00018000U)

#define DMA1_BASE (AHBPERIPH_BASE + 0x00000000U)
#define DMA1_Channel1_BASE (AHBPERIPH_BASE + 0x00000008U)
#define DMA1_Channel2_BASE (AHBPERIPH_BASE + 0x0000001CU)
#define DMA1_Channel3_BASE (AHBPERIPH_BASE + 0x00000030U)
#define DMA1_Channel4_BASE (AHBPERIPH_BASE + 0x00000044U)
#define DMA1_Channel5_BASE (AHBPERIPH_BASE + 0x00000058U)
#define DMA1_Channel6_BASE (AHBPERIPH_BASE + 0x0000006CU)
#define DMA1_Channel7_BASE (AHBPERIPH_BASE + 0x00000080U)
#define RCC_BASE (AHBPERIPH_BASE + 0x00001000U)
#define CRC_BASE (AHBPERIPH_BASE + 0x00003000U)

#define FLASH_R_BASE \
    (AHBPERIPH_BASE + 0x00002000U) /*!< Flash registers base address */
#define FLASHSIZE_BASE 0x1FFFF7E0U /*!< FLASH Size register base address */
#define UID_BASE 0x1FFFF7E8U /*!< Unique device ID register base address */
#define OB_BASE 0x1FFFF800U  /*!< Flash Option Bytes base address */

#define DBGMCU_BASE 0xE0042000U /*!< Debug MCU registers base address */

/* USB device FS */
#define USB_BASE       \
    (APB1PERIPH_BASE + \
     0x00005C00U) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR                                               \
    (APB1PERIPH_BASE +                                            \
     0x00006000U) /*!< USB_IP Packet Memory Area base address \ \ \
                   * \ \ \ \ \ \ \ \ \ \                      \ \ \
                   */

#define WWDG ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG ((IWDG_TypeDef *)IWDG_BASE)
#define SPI2 ((SPI_TypeDef *)SPI2_BASE)
#define USART2 ((USART_TypeDef *)USART2_BASE)
#define USART3 ((USART_TypeDef *)USART3_BASE)
#define I2C1 ((I2C_TypeDef *)I2C1_BASE)
#define I2C2 ((I2C_TypeDef *)I2C2_BASE)
#define USB ((USB_TypeDef *)USB_BASE)
#define CAN1 ((CAN_TypeDef *)CAN1_BASE)
#define BKP ((BKP_TypeDef *)BKP_BASE)
#define PWR ((PWR_TypeDef *)PWR_BASE)
#define AFIO ((AFIO_TypeDef *)AFIO_BASE)
#define EXTI ((EXTI_TypeDef *)EXTI_BASE)
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE ((GPIO_TypeDef *)GPIOE_BASE)
#define ADC1 ((ADC_TypeDef *)ADC1_BASE)
#define ADC2 ((ADC_TypeDef *)ADC2_BASE)
#define ADC12_COMMON ((ADC_Common_TypeDef *)ADC1_BASE)
#define TIM1 ((TIM_TypeDef *)TIM1_BASE)
#define TIM2 ((TIM_TypeDef *)TIM2_BASE)
#define SPI1 ((SPI_TypeDef *)SPI1_BASE)
#define USART1 ((USART_TypeDef *)USART1_BASE)
#define SDIO ((SDIO_TypeDef *)SDIO_BASE)
#define DMA1 ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_Channel1 ((DMA_Channel_TypeDef *)DMA1_Channel1_BASE)
#define DMA1_Channel2 ((DMA_Channel_TypeDef *)DMA1_Channel2_BASE)
#define DMA1_Channel3 ((DMA_Channel_TypeDef *)DMA1_Channel3_BASE)
#define DMA1_Channel4 ((DMA_Channel_TypeDef *)DMA1_Channel4_BASE)
#define DMA1_Channel5 ((DMA_Channel_TypeDef *)DMA1_Channel5_BASE)
#define DMA1_Channel6 ((DMA_Channel_TypeDef *)DMA1_Channel6_BASE)
#define DMA1_Channel7 ((DMA_Channel_TypeDef *)DMA1_Channel7_BASE)
#define RCC ((RCC_TypeDef *)RCC_BASE)
#define CRC ((CRC_TypeDef *)CRC_BASE)
#define FLASH ((FLASH_TypeDef *)FLASH_R_BASE)
#define OB ((OB_TypeDef *)OB_BASE)
#define DBGMCU ((DBGMCU_TypeDef *)DBGMCU_BASE)

#endif /* STM32F103C8T6_H */

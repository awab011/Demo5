## About
- Is a module that contains the essential board initialization code, functions, global variables, pins and GPIO macros.

## Files
![Adapter Source Files](attachment/src.png)
- Adapter Module is adapter.c and adapter.h files.

## Macros
### Pins Macro
```c
//// Pins ports and number ////
#define LED1_PIN GPIOE, GPIO_PIN_2
#define LED2_PIN GPIOE, GPIO_PIN_4
#define LED3_PIN GPIOE, GPIO_PIN_5
#define LED4_PIN GPIOC, GPIO_PIN_13
#define LED5_PIN GPIOC, GPIO_PIN_14
#define LED6_PIN GPIOC, GPIO_PIN_15

// Active Low Push buttons
#define PB1_PIN GPIOD, GPIO_PIN_8
#define PB2_PIN GPIOD, GPIO_PIN_9
#define PB3_PIN GPIOD, GPIO_PIN_10

// Digital pins
#define DIG1_PIN GPIOA, GPIO_PIN_10
#define DIG2_PIN GPIOB, GPIO_PIN_2
#define DIG3_PIN GPIOC, GPIO_PIN_0
#define DIG4_PIN GPIOB, GPIO_PIN_3
#define DIG5_PIN GPIOD, GPIO_PIN_5
#define DIG6_PIN GPIOD, GPIO_PIN_4
#define DIG7_PIN GPIOB, GPIO_PIN_4
#define DIG8_PIN GPIOA, GPIO_PIN_15
#define DIG9_PIN GPIOD, GPIO_PIN_11

// ADC GPIO PINS
#define ADC1_INP5 GPIOB, GPIO_PIN_1 // A6
#define ADC1_INN5 GPIOB, GPIO_PIN_0 // A5
#define ADC1_INP9 GPIOB, GPIO_PIN_0 // A5
#define ADC1_INP18 GPIOA, GPIO_PIN_4 // A4
#define ADC1_INN18 GPIOA, GPIO_PIN_5 // A3
#define ADC1_INP19 GPIOA, GPIO_PIN_5 // A3
#define ADC2_INN5 GPIOB, GPIO_PIN_0 // A5
#define ADC2_INN9 GPIOB, GPIO_PIN_0 // A5
#define ADC2_INN18 GPIOA, GPIO_PIN_5 // A3
#define ADC2_INP18 GPIOA, GPIO_PIN_4 // A4
#define ADC2_INP19 GPIOA, GPIO_PIN_5 // A3
#define ADC2_INP5 GPIOB, GPIO_PIN_1 // A6
#define ADC3_INP0 GPIOC, GPIO_PIN_3 // A1
#define ADC3_INP1 GPIOC, GPIO_PIN_2 // A2

// Analog Pins
#define A1_PIN ADC3_INP0
#define A2_PIN ADC3_INP1
#define A3_PIN ADC1_INP19
#define A4_PIN ADC1_INP18
#define A5_PIN ADC1_INP9
#define A6_PIN ADC1_INP5


// Timer GPIO Pins
/*
* @ Note: Need to see the timers map first to know all timers available, some ports can select one of multiple timers
* So the PWM ports that have multiple selection of timers will be like (PWMx_y), x is port number, y is the available timers you can choose
* marked as letters
*
* * Also be cautios when using same timer but different channels for different ports, not to change the timer parameters( All channels must have same timer parameters)
* */

// Only one channel can be used for a port
#define PWM1_A
//#define PWM1_B
//#define PWM1_C

#define PWM2_A
//#define PWM2_B
  
#define PWM3_A
//#define PWM3_B

#define PWM4_A

#define PWM5_A
//#define PWM5_B

#define PWM6_A
//#define PWM6_B

// QEI
#define TIM1_CH1_PIN GPIOA, GPIO_PIN_8 // QEI1_A -->TIM1
#define TIM1_CH2_PIN GPIOA, GPIO_PIN_9 // QEI1_B -->TIM1
#define QEI1_A_PIN TIM1_CH1_PIN
#define QEI1_B_PIN TIM1_CH2_PIN

#define TIM8_CH1_PIN GPIOC, GPIO_PIN_6 // QEI2_A -->TIM8
#define TIM8_CH2_PIN GPIOC, GPIO_PIN_7 // QEI2_B -->TIM8
#define QEI2_A_PIN TIM8_CH1_PIN
#define QEI2_B_PIN TIM8_CH2_PIN

// PWM ports
#ifdef PWM1_A
#define TIM2_CH4_PIN GPIOA, GPIO_PIN_3 // PWM1_A -->TIM2
#define PWM1_PIN TIM2_CH4_PIN // GPIOA, GPIO_PIN_3
#elif defined(PWM1_B)
#define TIM5_CH4_PIN GPIOA, GPIO_PIN_3 // PWM1_B -->TIM5
#define PWM1_PIN TIM5_CH4_PIN
#elif defined(PWM1_C)
#define TIM15_CH2_PIN GPIOA, GPIO_PIN_3 // PWM1_C -->TIM15
#define PWM1_PIN TIM15_CH2_PIN
#endif

#ifdef PWM2_A
#define TIM3_CH1_PIN GPIOA, GPIO_PIN_6 // PWM2_A -->TIM3
#define PWM2_PIN TIM3_CH1_PIN // GPIOA, GPIO_PIN_6
#elif defined(PWM2_B)
#define TIM13_CH1_PIN GPIOA, GPIO_PIN_6 // PWM2_B -->TIM13
#define PWM2_PIN TIM13_CH1_PIN
#endif

#ifdef PWM3_A
#define TIM2_CH1_PIN GPIOA, GPIO_PIN_0 // PWM3_A -->TIM2
#define PWM3_PIN TIM2_CH1_PIN // GPIOA, GPIO_PIN_0
#elif defined(PWM3_B)
#define TIM5_CH1_PIN GPIOA, GPIO_PIN_0 // PWM3_B -->TIM5
#define PWM3_PIN TIM5_CH1_PIN
#endif

#ifdef PWM4_A
#define TIM2_CH3_PIN GPIOB, GPIO_PIN_10 // PWM4_A -->TIM2
#define PWM4_PIN TIM2_CH3_PIN // GPIOB, GPIO_PIN_10
#endif

#ifdef PWM5_A
#define TIM3_CH4_PIN GPIOC, GPIO_PIN_9 // PWM5_A -->TIM3
#define PWM5_PIN TIM3_CH4_PIN // GPIOC, GPIO_PIN_9
#elif defined(PWM5_B)
#define TIM8_CH4_PIN GPIOC, GPIO_PIN_9 // PWM5_B -->TIM8
#define PWM5_PIN TIM8_CH4_PIN
#endif

#ifdef PWM6_A
#define TIM3_CH3_PIN GPIOC, GPIO_PIN_8 // PWM6_A -->TIM3
#define PWM6_PIN TIM3_CH3_PIN // GPIOC, GPIO_PIN_8
#elif defined(PWM6_B)
#define TIM8_CH3_PIN GPIOC, GPIO_PIN_8 // PWM6_B -->TIM8
#define PWM6_PIN TIM8_CH3_PIN
#endif

  
// Communication GPIOs
/*
* @Note:
* There is a mistake in the labeling system on the board for H7.1.0 version
* as the labeling is not the correct port number
*/

// UART
// UART5 labeled on the board H7.1.0 as UART1
#define UART5_RX_PIN GPIOD, GPIO_PIN_2
#define UART5_TX_PIN GPIOC, GPIO_PIN_12

// UART7 labeled on the board H7.1.0 as UART2
#define UART7_RX_PIN GPIOE, GPIO_PIN_7
#define UART7_TX_PIN GPIOE, GPIO_PIN_8

// UART4 labeled on the board H7.1.0 as UART3
#define UART4_RX_PIN GPIOD, GPIO_PIN_0
#define UART4_TX_PIN GPIOD, GPIO_PIN_1

// UART9 labeled on the board H7.1.0 as UART4
#define UART9_RX_PIN GPIOD, GPIO_PIN_14
#define UART9_TX_PIN GPIOD, GPIO_PIN_15

  
// SPI
/*
* @Note:
* Electronics said it is not designed well, but it is working
*/

// SPI2 labeled on the board H7.1.0 as SPI1
#define SPI2_MISO_PIN GPIOB, GPIO_PIN_14
#define SPI2_MOSI_PIN GPIOB, GPIO_PIN_1
#define SPI2_SCK_PIN GPIOD, GPIO_PIN_3
#define SPI2_NSS_PIN GPIOB, GPIO_PIN_9


// I2C

// I2C1 labeled on the board H7.1.0 as I1
#define I2C1_SDA_PIN GPIOB, GPIO_PIN_7
#define I2C1_SCL_PIN GPIOB, GPIO_PIN_8

// I2C5 labeled on the board H7.1.0 as I2
#define I2C5_SDA_PIN GPIOC, GPIO_PIN_10
#define I2C5_SDL_PIN GPIOC, GPIO_PIN_11


// FDCAN
// FDCAN1 Labeled as CAN2
#define FDCAN1_RX_PIN GPIOA, GPIO_PIN_11
#define FDCAN1_TX_PIN GPIOA, GPIO_PIN_12

// FDCAN2 Labeled as CAN1
#define FDCAN2_RX_PIN GPIOB, GPIO_PIN_5
#define FDCAN2_TX_PIN GPIOB, GPIO_PIN_6

// FDCAN3 Labeled as CAN3
#define FDCAN3_RX_PIN GPIOB, GPIO_PIN_12
#define FDCAN3_TX_PIN GPIOB, GPIO_PIN_13

  

// Ethernet
#define RMII_MDC_PIN GPIOC, GPIO_PIN_1
#define RMII_MDIO_PIN GPIOA, GPIO_PIN_2
#define RMII_TXD0_PIN GPIOB, GPIO_PIN_12
#define RMII_TXD1_PIN GPIOB, GPIO_PIN_13
#define RMII_TX_EN_PIN GPIOB, GPIO_PIN_11
#define RMII_RXD0_PIN GPIOC, GPIO_PIN_4
#define RMII_RXD1_PIN GPIOC, GPIO_PIN_5
#define RMII_CRSDV_PIN GPIOA, GPIO_PIN_7

```

### GPIO Macro
```c
///////// MACROs FOR INPUT AND OUTPUT MANIPULATION /////////
/*
* @Note: To use output or input features you need to initialize the GPIOs as output or input
*/

// LEDs output
#define led1 GPIOE_OUT->bit2
#define led2 GPIOE_OUT->bit4
#define led3 GPIOE_OUT->bit5
#define led4 GPIOC_OUT->bit13
#define led5 GPIOC_OUT->bit14
#define led6 GPIOC_OUT->bit15

//// OUTPUT macros ////
// Digital Pins
#define DIG1_OUT GPIOA_OUT->bit10
#define DIG2_OUT GPIOB_OUT->bit2
#define DIG3_OUT GPIOC_OUT->bit0
#define DIG4_OUT GPIOB_OUT->bit3
#define DIG5_OUT GPIOD_OUT->bit5
#define DIG6_OUT GPIOD_OUT->bit4
#define DIG7_OUT GPIOB_OUT->bit4
#define DIG8_OUT GPIOA_OUT->bit15
#define DIG9_OUT GPIOD_OUT->bit11

// ADC Pins
#define A1_OUT GPIOC_OUT->bit3
#define A2_OUT GPIOC_OUT->bit2
#define A3_OUT GPIOA_OUT->bit5
#define A4_OUT GPIOA_OUT->bit4
#define A5_OUT GPIOB_OUT->bit0
#define A6_OUT GPIOB_OUT->bit1

// PWM Pins
#define PWM1_OUT GPIOA_OUT->bit3
#define PWM2_OUT GPIOA_OUT->bit6
#define PWM3_OUT GPIOA_OUT->bit0
#define PWM4_OUT GPIOB_OUT->bit10
#define PWM5_OUT GPIOC_OUT->bit9
#define PWM6_OUT GPIOA_OUT->bit8

//// INPUT macros ////
#define PB1_IN GPIOD_IN->bit8
#define PB2_IN GPIOD_IN->bit9
#define PB3_IN GPIOD_IN->bit10

// Digital Pins
#define DIG1_IN GPIOA_IN->bit10
#define DIG2_IN GPIOB_IN->bit2
#define DIG3_IN GPIOC_IN->bit0
#define DIG4_IN GPIOB_IN->bit3
#define DIG5_IN GPIOD_IN->bit5
#define DIG6_IN GPIOD_IN->bit4
#define DIG7_IN GPIOB_IN->bit4
#define DIG8_IN GPIOA_IN->bit15
#define DIG9_IN GPIOD_IN->bit11

// ADC Pins
#define A1_IN GPIOC_IN->bit3
#define A2_IN GPIOC_IN->bit2
#define A3_IN GPIOA_IN->bit5
#define A4_IN GPIOA_IN->bit4
#define A5_IN GPIOB_IN->bit0
#define A6_IN GPIOB_IN->bit1

// PWM Pins
#define PWM1_IN GPIOA_IN->bit3
#define PWM2_IN GPIOA_IN->bit6
#define PWM3_IN GPIOA_IN->bit0
#define PWM4_IN GPIOB_IN->bit10
#define PWM5_IN GPIOC_IN->bit9
#define PWM6_IN GPIOA_IN->bit8
  
////////// END OF OUTPUT INPUT MACROs ////////
```
- GPIO macro can be used for input reading and output manipulation.
- ![Note] that you need to initialize the pin first.
## Global Variables
```c
//** Global variables **//
// RBC Libraries global variables
extern R6091U_t IMU;
extern PSxBT_t ps4;
extern H7_EXTIHandler_s EXTI_Handler;
extern PWMEnc_t enc1,enc2,enc3,enc4;
```

## Functions
```c
void initialize(void);
void CAN_PROCESS(PACKET_t packet_src);
```

### initialize()
- This function is called in the main function, after system [H7_system_init](H7_system.md#h7_system_init).
- All initialization functions are there but commented. Not all.

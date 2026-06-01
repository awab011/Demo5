/**
 * @file    apriorities.h
 * @brief   System Interrupt priority
 * @author  Mohammed Abdulalem
 * @date    2026-2-12
 *
 * @details All interrupt NVIC priorities are determined by
 * the folowing macros
 *
 *
 * @note
 * - 0->15 Higher->Lower poriority
 * - Priorities should be choosed wisely as this might cause the board
 * to jam, very high repeated request peripherals should have low priority
 */
#ifndef SRC_PLATFORM_PRIORITIES_H_
#define SRC_PLATFORM_PRIORITIES_H_




// IT0, is were high priority tasks are assigned
#define H7_FDCAN1_IT0_IRQ_PRIO				8
#define H7_FDCAN2_IT0_IRQ_PRIO				8
#define H7_FDCAN3_IT0_IRQ_PRIO				8
// IT1, is were low priority tasks are assigned
#define H7_FDCAN1_IT1_IRQ_PRIO				13
#define H7_FDCAN2_IT1_IRQ_PRIO				13
#define H7_FDCAN3_IT1_IRQ_PRIO				13
// Clock calibration in case of any drift or noise
#define H7_FDCAN_CAL_IRQ_PRIO				13


#define H7_I2C1_EVENT_IRQ_PRIO				10
#define H7_I2C1_ERROR_IRQ_PRIO				8

#define H7_I2C5_EVENT_IRQ_PRIO				10
#define H7_I2C5_ERROR_IRQ_PRIO				8

// SPI
#define H7_SPI2_IRQ_PRIO					10
// SPI DMA
#define H7_SPI2_TX_DMA_PRIO					8
#define H7_SPI2_RX_DMA_PRIO					8

// UART
#define H7_UART4_IRQ_PRIO					12
#define H7_UART5_IRQ_PRIO					12
#define H7_UART7_IRQ_PRIO					12
#define H7_UART9_IRQ_PRIO					12
// UART DMA
#define H7_UART4_TX_DMA_IRQ_PRIO			5
#define H7_UART4_RX_DMA_IRQ_PRIO			5

#define H7_UART5_TX_DMA_IRQ_PRIO			5
#define H7_UART5_RX_DMA_IRQ_PRIO			5

#define H7_UART7_TX_DMA_IRQ_PRIO			5
#define H7_UART7_RX_DMA_IRQ_PRIO			5

#define H7_UART9_TX_DMA_IRQ_PRIO			5
#define H7_UART9_RX_DMA_IRQ_PRIO			5
// Encoder QEI1 timer
#define H7_TIM1_UPDATE_IRQ_PRIO					7
#define H7_TIM1_TRG_COM_IRQ_PRIO			10
#define H7_TIM1_COMPARE_CAPTURE_IRQ_PRIO	7
// Encoder QEI2 timer
#define H7_TIM8_TIM12_BREAK_IRQ_PRIO		5
#define H7_TIM8_TIM13_UPDATE_IRQ_PRIO		7
#define H7_TIM8_TIM14_TRG_COM_IRQ_PRIO		10
#define H7_TIM8_COMPARE_CAPTURE_IRQ_PRIO	7

#define H7_TIM2_IRQ_PRIO					5
#define H7_TIM3_IRQ_PRIO					5
#define H7_TIM4_IRQ_PRIO					5
#define H7_TIM5_IRQ_PRIO					5
#define H7_TIM6_DAC_IRQ_PRIO				2
#define H7_TIM7_IRQ_PRIO					5
#define H7_TIM12_IRQ_PRIO					5
#define H7_TIM15_IRQ_PRIO					5
#define H7_TIM16_IRQ_PRIO					5
#define H7_TIM17_IRQ_PRIO					5
#define H7_TIM23_IRQ_PRIO					5
#define H7_TIM24_IRQ_PRIO					5









#endif /* SRC_PLATFORM_PRIORITIES_H_ */

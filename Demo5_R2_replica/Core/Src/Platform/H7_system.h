/*
 * H7_system.h
 *
 *  Created on: Oct 19, 2025
 *      Author: mua
 */

#ifndef SRC_H7_SYSTEM_H_
#define SRC_H7_SYSTEM_H_

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

// This should be written before declaring any variable that is reading using DMA
#define DMA_BUFFER __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))


#define 			CYCLES_PER_us (SystemCoreClock / 1000000)		// Cycles per Micro seconds
#ifndef _STDBOOL_H
#define 			ON	1
#define 			OFF	0
#endif

#define H7_ERR_HANDLE(__SYSTEM_HANDLER__, __STATUS__) \
	__SYSTEM_HANDLER__.status.last_error = __STATUS__; \
	__SYSTEM_HANDLER__.status.error_count++

#define Bit0 		((uint8_t)0b00000001)
#define Bit1		((uint8_t)0b00000010)
#define Bit2		((uint8_t)0b00000100)
#define Bit3		((uint8_t)0b00001000)
#define Bit4		((uint8_t)0b00010000)
#define Bit5		((uint8_t)0b00100000)
#define Bit6		((uint8_t)0b01000000)
#define Bit7		((uint8_t)0b10000000)

/* it is not recomended to use these macros:
 * SIGN
 * Llim
 * Ulim
 * chlim
 * ch360
 * ch180
 * SQ
 * as they are risky to use if you do not understand
 * how the compiler will deal with them.
 */
#define SIGN(a)		((a < 0.0)? -1 : 1)
#define Llim(a, b)	a = ((a <= -b)? -b : a)
#define Ulim(x, c)	x = ((x >= c) ?  c : x)
#define chlim(a, b) if(a >= b){a = b;} else if(a <= -b){a = -b;}
#define ch360(a) 	((a >= 360.0)? (a -= 360.0) : (a < 0.0)   ? (a += 360.0) : a)
#define ch180(a) 	((a >= 180.0)? (a -= 360.0) : (a < -180.0)? (a += 360.0) : a)
#define SQ(x)		((x)*(x))
#define COS_30_DEG	0.86602540378
#define COS_45_DEG	0.70710678118
#define SIN_30_DEG	0.5
#define SIN_45_DEG	0.70710678118
#define SIN_60_DEG	0.86602540378

///// Signed types /////
typedef __int64_t 				s64;
typedef __int32_t 				s32;
typedef __int16_t 				s16;
typedef __int8_t				s8;

// Constant signed types
typedef const __int64_t 		sc64;
typedef const __int32_t 		sc32;
typedef const __int16_t 		sc16;
typedef const __int8_t 			sc8;

// Volatile signed types
typedef volatile __int64_t 		vs64;
typedef volatile __int32_t 		vs32;
typedef volatile __int16_t		vs16;
typedef volatile __int8_t  		vs8;


///// Unsigned types /////
typedef __uint64_t				u64;
typedef __uint32_t				u32;
typedef __uint16_t 				u16;
typedef __uint8_t				u8;

// Unsigned constant types
typedef const __uint64_t 		uc64;
typedef const __uint32_t 		uc32;
typedef const __uint16_t 		uc16;
typedef const __uint8_t 		uc8;

// Volatile unsigned types
typedef volatile __uint64_t  	vu64;
typedef volatile __uint32_t  	vu32;
typedef volatile __uint16_t 	vu16;
typedef volatile __uint8_t  	vu8;






enum {PENDING_SYNC = 0, CONFIRMING_SYNC, IN_SYNC};


typedef struct{
	union{
		u32 reg_32; // A register container
		struct{
			unsigned bit0		:	1;
			unsigned bit1		:	1;
			unsigned bit2		:	1;
			unsigned bit3		:	1;
			unsigned bit4		:	1;
			unsigned bit5		:	1;
			unsigned bit6		:	1;
			unsigned bit7		:	1;
			unsigned bit8		:	1;
			unsigned bit9		:	1;
			unsigned bit10		:	1;
			unsigned bit11		:	1;
			unsigned bit12		:	1;
			unsigned bit13		:	1;
			unsigned bit14		:	1;
			unsigned bit15		:	1;
			unsigned bit16		:	1;
			unsigned bit17		:	1;
			unsigned bit18		:	1;
			unsigned bit19		:	1;
			unsigned bit20		:	1;
			unsigned bit21		:	1;
			unsigned bit22		:	1;
			unsigned bit23		:	1;
			unsigned bit24		:	1;
			unsigned bit25		:	1;
			unsigned bit26		:	1;
			unsigned bit27		:	1;
			unsigned bit28		:	1;
			unsigned bit29		:	1;
			unsigned bit30		:	1;
			unsigned bit31		:	1;
		};
	};
} H7_word_s;

typedef struct{
	union{
		u16 reg_16; // A 16-bit register container
		struct{
			unsigned bit0		:	1;
			unsigned bit1		:	1;
			unsigned bit2		:	1;
			unsigned bit3		:	1;
			unsigned bit4		:	1;
			unsigned bit5		:	1;
			unsigned bit6		:	1;
			unsigned bit7		:	1;
			unsigned bit8		:	1;
			unsigned bit9		:	1;
			unsigned bit10		:	1;
			unsigned bit11		:	1;
			unsigned bit12		:	1;
			unsigned bit13		:	1;
			unsigned bit14		:	1;
			unsigned bit15		:	1;
		};
	};
} H7_half_word_s;


typedef enum{
	/**************** System states ****************/
	H7_STATE_NULL,
	H7_OK,		// for system and other things
	H7_PERIPH_OK, // For Peripherals
	H7_INV_PORT,
	H7_INV_MODE,
	/**************** GPIO states ****************/
	H7_GPIO_PORT_INV,

	/**************** UART states ****************/
	H7_UART_NULL_PTR,
	H7_UART_PORT_INV,
	H7_UART_INIT_ERR,
	H7_UART_DMA_INIT_ERR,

	/**************** Timer states ****************/
	H7_PWM_CHANNEL_OK,
	H7_TIM_NULL_PTR,
	H7_TIM_INVALID_TIMER,
	H7_TIM_INIT_ERR,
	H7_TIM_PWM_INIT_ERR,
	H7_TIM_TIMEOUT_ERR,

	// Configuration Errors
	H7_TIM_MASTER_CONF_ERR,
	H7_TIM_CLKSOURCE_CONF_ERR,
	H7_PWM_CONFIG_CHANNEL_ERR,
	H7_TIM_START_Failed,
	H7_TIM_START_IT_Failed,
	H7_PWM_START_FAILED,

	/**************** FDCAN states ****************/
	H7_FDCAN_FILTER_OK,
	H7_FDCAN_NULL_PTR,
	H7_FDCAN_INVALID_PORT,
	H7_FDCAN_INIT_ERR,
	H7_FDCAN_START_ERR,
	H7_FDCAN_FIFO_ACTICATE_ERR,
	H7_FDCAN_TIMEOUT_ERR,
	H7_FDCAN_FILTER_INIT_ERR,
	H7_FDCAN_MEM_RAM_OVERSIZE,
	H7_FDCAN_MSGSEND_ERR,
	H7_FDCAN_TX_FIFO_FULL,

	/**************** I2C states ****************/
	H7_I2C_NULL_PTR,
	H7_I2C_INVALID_PORT,
	H7_I2C_INVALID_SPEED,
	H7_I2C_INIT_ERR,
	H7_I2C_DMA_INIT_ERR,
	H7_I2C_TIMEOUT_ERR,

	// Configuration Errors
	H7_I2C_AN_FILTER_CONFIG_ERR, // Analog filter
	H7_I2C_DG_FILTER_CONFIG_ERR, // Digital filter
	H7_I2C_START_IT_Failed,

	/**************** QEI states ****************/
	H7_QEI_INVALID_PORT,
	H7_QEI_INIT_ERR,
	/**************** EXTI states ****************/
	H7_EXTI_NULL_PTR,
	H7_EXTI_INVALID_PIN
} H7_state_e;



typedef struct {

	//** System State **//
    struct {
        H7_state_e          system;           // Overall system state (H7_OK, H7_ERROR, etc.)
        H7_state_e          last_error;     // Last error that occurred
        u16           		error_count;    // Total errors since boot
        float               cpu_load;       // CPU load percentage (0-100%)
        u32            		free_heap;      // Available heap memory (bytes)
    } status;

    struct {
        vu32            	(*get_tick_us)(void);     // Microsecond timestamp, use only for micro seconds operations
        u32            		(*get_tick_ms)(void);     // Millisecond timestamp
        u32            		(*get_tick_s)(void);     	// second timestamp
        void                (*delay_us)(u32);    // Blocking microsecond delay
        osStatus_t                (*delay_ms)(u32);    // Blocking millisecond delay
    } time;

    // Function related to core
    void          	(*reset)(void);   // Function pointer to soft reset
} H7_System_s;

extern H7_System_s H7SYSTEM;

//////////////////////////////// Initializing and Configuration Functions ////////////////////////////////
void H7_system_init(void);
void H7_reset(void);


//////////////////////////////// Timing Functions ////////////////////////////////
void DWT_delay_us(u32 us);
vu32 DWT_GetTick_us(void);
#endif /* SRC_H7_SYSTEM_H_ */

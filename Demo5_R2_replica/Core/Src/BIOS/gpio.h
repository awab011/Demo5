/**
 * @file    gpio.h
 * @brief   GPIO header driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup GPIO
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_



//** Includes **//
#include "../Platform/H7_system.h"

//------------------- Enumeration -------------------//


//------------------- Structures -------------------//


//------------------- Macros and Global Variables -------------------//


//** Macros **//
#define GPIOA_OUT 	((volatile H7_word_s *) &GPIOA->ODR)
#define GPIOB_OUT 	((volatile H7_word_s *) &GPIOB->ODR)
#define GPIOC_OUT 	((volatile H7_word_s *) &GPIOC->ODR)
#define GPIOD_OUT 	((volatile H7_word_s *) &GPIOD->ODR)
#define GPIOE_OUT 	((volatile H7_word_s *) &GPIOE->ODR)
#define GPIOF_OUT 	((volatile H7_word_s *) &GPIOF->ODR)
#define GPIOG_OUT 	((volatile H7_word_s *) &GPIOG->ODR)
#define GPIOH_OUT 	((volatile H7_word_s *) &GPIOH->ODR)
//#define GPIOI_OUT 	((H7_word_s *) &GPIOI->ODR)
#define GPIOJ_OUT 	((volatile H7_word_s *) &GPIOJ->ODR)
#define GPIOK_OUT 	((volatile H7_word_s *) &GPIOK->ODR)

#define GPIOA_IN	((volatile H7_word_s *) &GPIOA->IDR)
#define GPIOB_IN	((volatile H7_word_s *) &GPIOB->IDR)
#define GPIOC_IN	((volatile H7_word_s *) &GPIOC->IDR)
#define GPIOD_IN	((volatile H7_word_s *) &GPIOD->IDR)
#define GPIOE_IN	((volatile H7_word_s *) &GPIOE->IDR)
#define GPIOF_IN	((volatile H7_word_s *) &GPIOF->IDR)
#define GPIOG_IN	((volatile H7_word_s *) &GPIOG->IDR)
#define GPIOH_IN	((volatile H7_word_s *) &GPIOH->IDR)
//#define GPIOI_IN	((H7_word_s *) &GPIOI->IDR)
#define GPIOJ_IN	((volatile H7_word_s *) &GPIOJ->IDR)
#define GPIOK_IN	((volatile H7_word_s *) &GPIOK->IDR)


//** Global variables **//


//------------------- Function Declaration -------------------//
H7_state_e H7_gpio_PinInit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, u32 Mode, u32 GPIO_Speed,  u32 GPIO_PuPd);



#endif /* SRC_GPIO_H_ */

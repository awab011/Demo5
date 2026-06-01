/**
 * @file    gpio.c
 * @brief   GPIO driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup GPIO
 */



//** Includes **//
#include "gpio.h"

//** Global variables **//


//------------------- Function Defination -------------------//
H7_state_e H7_gpio_PinInit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, u32 Mode, u32 GPIO_Speed,  u32 GPIO_PuPd){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOH_CLK_ENABLE();
	switch((u32)GPIOx){

	case GPIOA_BASE:
		__HAL_RCC_GPIOA_CLK_ENABLE();
		break;
	case GPIOB_BASE:
		__HAL_RCC_GPIOB_CLK_ENABLE();
		break;
	case GPIOC_BASE:
		__HAL_RCC_GPIOC_CLK_ENABLE();
		break;
	case GPIOD_BASE:
		__HAL_RCC_GPIOD_CLK_ENABLE();
		break;
	case GPIOE_BASE:
		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case GPIOF_BASE:
		__HAL_RCC_GPIOF_CLK_ENABLE();
		break;
	case GPIOG_BASE:
		__HAL_RCC_GPIOG_CLK_ENABLE();
		break;
	case GPIOH_BASE:
		__HAL_RCC_GPIOH_CLK_ENABLE();
		break;
//	case GPIOI_BASE:   /* Need to search about port I */
//		__HAL_RCC_GPIOI_CLK_ENABLE();
//		break;
	case GPIOJ_BASE:
		__HAL_RCC_GPIOJ_CLK_ENABLE();
		break;
	case GPIOK_BASE:
		__HAL_RCC_GPIOK_CLK_ENABLE();
		break;
	default:
		return H7_GPIO_PORT_INV;
	}

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin 	= GPIO_Pin;
	GPIO_InitStruct.Mode 	= Mode;
	GPIO_InitStruct.Speed 	= GPIO_Speed;
	GPIO_InitStruct.Pull 	= GPIO_PuPd;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);


	return H7_PERIPH_OK;
}

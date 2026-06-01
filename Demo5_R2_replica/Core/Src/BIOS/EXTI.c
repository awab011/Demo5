/**
 * @file    EXTI.c
 * @brief   EXTI driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup GPIO
 */



//** Includes **//
#include "EXTI.h"


//** Global variables **//


//------------------- Function Defination -------------------//
H7_state_e H7_EXTI_init(H7_EXTIHandler_s *H7EXTI, u16 GPIO_PIN, u32 PreemptPriority){

	if(H7EXTI == NULL){
		return H7_EXTI_NULL_PTR;
	}
	H7EXTI->GPIO_EXTI_PIN = GPIO_PIN;
	IRQn_Type IRQn;

	if(GPIO_PIN == GPIO_PIN_0){

			IRQn = EXTI0_IRQn;

		}
		else if(GPIO_PIN == GPIO_PIN_1){

			IRQn = EXTI1_IRQn;

		}
		else if(GPIO_PIN == GPIO_PIN_2){

			IRQn = EXTI2_IRQn;

		}
		else if(GPIO_PIN == GPIO_PIN_3){

			IRQn = EXTI3_IRQn;

		}
		else if(GPIO_PIN == GPIO_PIN_4){

			IRQn = EXTI4_IRQn;

		}
		else if(GPIO_PIN == GPIO_PIN_5 || GPIO_PIN == GPIO_PIN_6 || GPIO_PIN == GPIO_PIN_7 || GPIO_PIN == GPIO_PIN_8 || GPIO_PIN == GPIO_PIN_9){

			IRQn = EXTI9_5_IRQn;

		}
		else if(GPIO_PIN == GPIO_PIN_10 || GPIO_PIN == GPIO_PIN_11 || GPIO_PIN == GPIO_PIN_12 || GPIO_PIN == GPIO_PIN_13 || GPIO_PIN == GPIO_PIN_14 || GPIO_PIN == GPIO_PIN_15){

			IRQn = EXTI15_10_IRQn;
		}else{
			H7EXTI->state = H7_EXTI_INVALID_PIN;
			return H7_EXTI_INVALID_PIN;
		}

		HAL_NVIC_SetPriority(IRQn, PreemptPriority, 0);

		HAL_NVIC_EnableIRQ(IRQn);

		H7EXTI->state = H7_PERIPH_OK;
		return H7_PERIPH_OK;
}









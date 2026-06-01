/*
 * PWMEncoder.c
 *
 *  Created on: May 9, 2024
 *      Author: Saif
 */

#include "PWMEncoder.h"

void PWMEncoder_Angle(TIM_HandleTypeDef* htimx, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, PWMEnc_t *enc){

	enc->enc_GPIOX = GPIOx;

	enc->enc_PIN = GPIO_Pin;

	if((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET){

		enc->fullCount = htimx->Instance->CNT - enc->prevStart;

		enc->start = htimx->Instance->CNT;

	}


	if((GPIOx->IDR & GPIO_Pin) == (uint32_t)GPIO_PIN_RESET){

		enc->prevStart = enc->start;

		enc->end = htimx->Instance->CNT;

	}

	if(enc->end > enc->start){

		enc->diffrence = enc->end - enc->start;

	}

	else{

		enc->diffrence = (0xFFFF + enc->end) - enc->start;

	}

}

void PWMEncoder_Angle_Update(PWMEnc_t *enc){


		enc->dutyc = 100 * enc->diffrence / (float)enc->fullCount;

		if(enc->dutyc >= 0.3884438 && enc->dutyc <= 99.8057781){

			enc->Angle = 360 * (enc->dutyc - 0.3884438) / (99.805778 - 0.3884438);

			enc->error = 0;
		}


		else if(enc->diffrence < 0.3884438 * enc->fullCount){

			enc->error = 1;

		}

	}








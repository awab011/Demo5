/*
 * PWMEncoder.h
 *
 *  Created on: May 9, 2024
 *      Author: mohds
 *  User guide: https://tartan-acai-d8a.notion.site/PWM-Encoder-a6189cfe644c432bbd8ce8043ef7e6d0?pvs=4
 *
 */

#ifndef SRC_PWMENCODER_PWMENCODER_H_
#define SRC_PWMENCODER_PWMENCODER_H_

#include "../../BIOS/BIOS.h"

typedef struct{

	GPIO_TypeDef* enc_GPIOX;

	uint16_t enc_PIN;

	TIM_HandleTypeDef* enc_Tim;

	int start;

	int end;

	int diffrence;

	int prevStart;

	float fullCount;

	int error;

	float dutyc;

	float Angle;


}PWMEnc_t;


void PWMEncoder_Angle(TIM_HandleTypeDef* htimx, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, PWMEnc_t *enc );
void PWMEncoder_Angle_Update(PWMEnc_t *enc);

#endif /* SRC_PWMENCODER_PWMENCODER_H_ */

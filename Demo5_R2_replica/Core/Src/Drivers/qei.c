/**
 * @file    qei.c
 * @brief   Quadrature Encoder Interface driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Timers
 */


//** Includes **//
#include "qei.h"


//** Global variables **//
H7_QEIHandler_s QEI1;
H7_QEIHandler_s QEI2;

//------------------- Function Defination -------------------//
H7_state_e H7_QEI_init(H7_QEIHandler_s *h7qei, TIM_HandleTypeDef *htim){
	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	h7qei->htimer = htim;
	if(h7qei->htimer == &htim1){
		h7qei->htimer->Instance = TIM1;
	} else if(h7qei->htimer == &htim8){
		h7qei->htimer->Instance = TIM8;
	}else{
		h7qei->state = H7_QEI_INVALID_PORT;
		return H7_QEI_INVALID_PORT;
	}

	h7qei->htimer->Init.Prescaler	= 0;
	h7qei->htimer->Init.CounterMode	= TIM_COUNTERMODE_UP;
	h7qei->htimer->Init.Period		= 65535;
	h7qei->htimer->Init.ClockDivision	= TIM_CLOCKDIVISION_DIV1;
	h7qei->htimer->Init.RepetitionCounter	= 0;
	h7qei->htimer->Init.AutoReloadPreload	= TIM_AUTORELOAD_PRELOAD_ENABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 5;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 5;
	if (HAL_TIM_Encoder_Init(h7qei->htimer, &sConfig) != HAL_OK)
	{
		h7qei->state = H7_QEI_INIT_ERR;
		return H7_QEI_INIT_ERR;
	}
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(h7qei->htimer, &sMasterConfig) != HAL_OK)
  {
//	Error_Handler();
  }


	HAL_TIM_Encoder_Start_IT(h7qei->htimer, TIM_CHANNEL_ALL);

	h7qei->state = H7_OK;
	return H7_OK;

}


vs32 H7_QEI_read(H7_QEIHandler_s *h7qei){
	return h7qei->count;
}

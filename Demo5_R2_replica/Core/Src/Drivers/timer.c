 /**
 * @file    timer.c
 * @brief   timer driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Timers
 */


//** Includes **//
#include "timer.h"

//** Global variables **//
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
TIM_HandleTypeDef htim23;
TIM_HandleTypeDef htim24;

H7_TimerHandler_s h7timer1;
H7_TimerHandler_s h7timer2;
H7_TimerHandler_s h7timer3;
H7_TimerHandler_s h7timer4;
H7_TimerHandler_s h7timer5;
H7_TimerHandler_s h7timer6;
H7_TimerHandler_s h7timer7;
H7_TimerHandler_s h7timer8;
H7_TimerHandler_s h7timer12;
H7_TimerHandler_s h7timer13;
H7_TimerHandler_s h7timer14;
H7_TimerHandler_s h7timer15;
H7_TimerHandler_s h7timer16;
H7_TimerHandler_s h7timer17;
H7_TimerHandler_s h7timer23;
H7_TimerHandler_s h7timer24;


//------------------- Function Defination -------------------//
H7_state_e H7_TIMx_init_strcut(H7_TimerHandler_s *h7timer, TIM_HandleTypeDef *htimer){
	if(h7timer == NULL || htimer == NULL){
		return H7_TIM_NULL_PTR;
	}
	h7timer->htimer = htimer;
	h7timer->status = H7_PERIPH_OK;
	return H7_PERIPH_OK;
}
/*
 * APB1: TIM2/TIM3/TIM4/TIM5/TIM6/TIM7/TIM12/TIM13/TIM14/LPTIM1/TIM23/TIM24 ->  275MHz
 * APB2: 275MHz TIM1/TIM8/TIM15/TIM16/TIM17 -> 275MHz
 * APB4: LPTIM2/LPTIM3/LPTIM4/LPTIM5
 *
 * 						  ***************TO FIND Tout********************
 *
 * 						     Tout = ((period)*(prescaler))/Tclk
 *
 * 						  ***********************************************
 */


H7_state_e H7_TIMx_init(H7_TimerHandler_s *h7timer, u32 prescaler, u32 period){


	if(h7timer->htimer == &htim1){
		h7timer->htimer->Instance = TIM1;
	}else if(h7timer->htimer == &htim2){
		h7timer->htimer->Instance = TIM2;
	}else if(h7timer->htimer == &htim3){
		h7timer->htimer->Instance = TIM3;
	}else if(h7timer->htimer == &htim4){
		h7timer->htimer->Instance = TIM4;
	}else if(h7timer->htimer == &htim5){
		h7timer->htimer->Instance = TIM5;
	}else if(h7timer->htimer == &htim6){
		h7timer->htimer->Instance = TIM6;
	}else if(h7timer->htimer == &htim7){
		h7timer->htimer->Instance = TIM7;
	}else if(h7timer->htimer == &htim8){
		h7timer->htimer->Instance = TIM8;
	}else if(h7timer->htimer == &htim12){
		h7timer->htimer->Instance = TIM12;
	}else if(h7timer->htimer == &htim13){
		h7timer->htimer->Instance = TIM13;
	}else if(h7timer->htimer == &htim14){
		h7timer->htimer->Instance = TIM14;
	}else if(h7timer->htimer == &htim15){
		h7timer->htimer->Instance = TIM15;
	}else if(h7timer->htimer == &htim16){
		h7timer->htimer->Instance = TIM16;
	}else if(h7timer->htimer == &htim17){
		h7timer->htimer->Instance = TIM17;
	}else if(h7timer->htimer == &htim23){
		h7timer->htimer->Instance = TIM23;
	}else if(h7timer->htimer == &htim24){
		h7timer->htimer->Instance = TIM24;
	}else{
		h7timer->status = H7_TIM_INVALID_TIMER;
		return H7_TIM_INVALID_TIMER;
	}

	h7timer->htimer->Init.Prescaler		= prescaler - 1;
	h7timer->htimer->Init.Period		= period - 1;
	h7timer->htimer->Init.CounterMode	= TIM_COUNTERMODE_UP;
	h7timer->htimer->Init.ClockDivision	= TIM_CLOCKDIVISION_DIV1;

	if (HAL_TIM_Base_Init(h7timer->htimer) != HAL_OK)
	{
		h7timer->status = H7_TIM_INIT_ERR;
		return H7_TIM_INIT_ERR;
	}

	if(HAL_TIM_Base_Start_IT(h7timer->htimer) != HAL_OK){
		h7timer->status = H7_TIM_START_IT_Failed;
		return H7_TIM_START_IT_Failed;
	}
	h7timer->status = H7_PERIPH_OK;
	return H7_PERIPH_OK;

}

/* H7.1.0 board is using 6 PWM ports which are connected to timers TIM2/TIM3/TIM5/TIM8/TIM13/TIM15,
 * However in this library version H7L.1.0 uses TIM2/TIM3 only
 *
 */



H7_state_e H7_PWM_TIM_config(H7_TimerHandler_s *h7timer, u32 prescaler, u32 period){
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	if(h7timer->htimer == &htim2){
		h7timer->htimer->Instance = TIM2;
	}else if(h7timer->htimer == &htim3){
		h7timer->htimer->Instance = TIM3;
	}else if(h7timer->htimer == &htim5){
		h7timer->htimer->Instance = TIM5;
	}else if(h7timer->htimer == &htim8){
		h7timer->htimer->Instance = TIM8;
	}else if(h7timer->htimer == &htim13){
		h7timer->htimer->Instance = TIM13;
	}else if(h7timer->htimer == &htim15){
		h7timer->htimer->Instance = TIM15;
	}else{
		h7timer->status = H7_TIM_INVALID_TIMER;
		return H7_TIM_INVALID_TIMER;
	}

	h7timer->htimer->Init.Prescaler				= prescaler - 1;
	h7timer->htimer->Init.Period				= period - 1;
	h7timer->htimer->Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
	h7timer->htimer->Init.CounterMode			= TIM_COUNTERMODE_UP;
	h7timer->htimer->Init.ClockDivision			= TIM_CLOCKDIVISION_DIV1;

	if (HAL_TIM_Base_Init(h7timer->htimer) != HAL_OK)
	{
		h7timer->status = H7_TIM_INIT_ERR;
		return H7_TIM_INIT_ERR;
	}

	if (HAL_TIM_PWM_Init(h7timer->htimer) != HAL_OK)
	{
		h7timer->status = H7_TIM_PWM_INIT_ERR;
		return H7_TIM_PWM_INIT_ERR;
	}

	sMasterConfig.MasterOutputTrigger 	= TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(h7timer->htimer, &sMasterConfig) != HAL_OK)
	{
		h7timer->status = H7_TIM_MASTER_CONF_ERR;
		return H7_TIM_MASTER_CONF_ERR;
	}
	h7timer->status = H7_PERIPH_OK;
	return H7_PERIPH_OK;
}

H7_state_e H7_PWM_channel_config(H7_TimerHandler_s *h7timer, u32 channel){
	TIM_OC_InitTypeDef sConfigOC = {0};

	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= 0;
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_ENABLE;

	if (HAL_TIM_PWM_ConfigChannel(h7timer->htimer, &sConfigOC, channel) != HAL_OK)
	{
		h7timer->status = H7_PWM_CONFIG_CHANNEL_ERR;
		return H7_PWM_CONFIG_CHANNEL_ERR;
	}

	// Configure the pins
	HAL_TIM_MspPostInit(h7timer->htimer);

	if(HAL_TIM_PWM_Start(h7timer->htimer, channel) != HAL_OK){
		h7timer->status = H7_PWM_START_FAILED;
		return H7_PWM_START_FAILED;
	}

	h7timer->status = H7_PWM_CHANNEL_OK;
	return H7_PWM_CHANNEL_OK;
}

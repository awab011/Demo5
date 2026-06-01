/**
 * @file    timer.h
 * @brief   Timer header driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Timers
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_


//** Includes **//
#include "../Platform/H7_system.h"

//------------------- Enumeration -------------------//


//------------------- Structures -------------------//
typedef struct{
	TIM_HandleTypeDef *htimer;
	H7_state_e status;
} H7_TimerHandler_s;


//------------------- Macros and Global Variables -------------------//


//** Macros **//
// To set the PWM HIGH period
#define H7_SET_COMPARE(TIM_HANDLER, CHANNEL, COMPARE)\
	(((CHANNEL) == TIM_CHANNEL_1)? ((TIM_HANDLER).htimer->Instance->CCR1 = (COMPARE)):\
	 ((CHANNEL) == TIM_CHANNEL_2)? ((TIM_HANDLER).htimer->Instance->CCR2 = (COMPARE)):\
	 ((CHANNEL) == TIM_CHANNEL_3)? ((TIM_HANDLER).htimer->Instance->CCR3 = (COMPARE)):\
	 ((CHANNEL) == TIM_CHANNEL_4)? ((TIM_HANDLER).htimer->Instance->CCR4 = (COMPARE)): 0)


//** Global variables **//
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim24;

extern H7_TimerHandler_s h7timer1;
extern H7_TimerHandler_s h7timer2;
extern H7_TimerHandler_s h7timer3;
extern H7_TimerHandler_s h7timer4;
extern H7_TimerHandler_s h7timer5;
extern H7_TimerHandler_s h7timer6;
extern H7_TimerHandler_s h7timer7;
extern H7_TimerHandler_s h7timer8;
extern H7_TimerHandler_s h7timer12;
extern H7_TimerHandler_s h7timer13;
extern H7_TimerHandler_s h7timer14;
extern H7_TimerHandler_s h7timer15;
extern H7_TimerHandler_s h7timer16;
extern H7_TimerHandler_s h7timer17;
extern H7_TimerHandler_s h7timer23;
extern H7_TimerHandler_s h7timer24;

//------------------- Function Declaration -------------------//
H7_state_e H7_TIMx_init_strcut(H7_TimerHandler_s *h7timer, TIM_HandleTypeDef *htimer);
H7_state_e H7_TIMx_init(H7_TimerHandler_s *h7timer, u32 prescaler, u32 period);
H7_state_e H7_PWM_TIM_config(H7_TimerHandler_s *h7timer, u32 prescaler, u32 period);
H7_state_e H7_PWM_channel_config(H7_TimerHandler_s *h7timer, u32 channel);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



#endif /* SRC_TIMER_H_ */

/**
 * @file    qei.h
 * @brief   Quadrature Encoder Interface header driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Timers
 */


#ifndef SRC_DRIVERS_QEI_H_
#define SRC_DRIVERS_QEI_H_


//** Includes **//
#include "../Platform/H7_system.h"
#include "timer.h"

//------------------- Enumeration -------------------//


//------------------- Structures -------------------//
typedef struct{
	TIM_HandleTypeDef *htimer;
	vs32 count;
//	union{
//		int32_t count;
//		struct{
//			uint16_t poscnt;
//			int16_t  signbit;
//		};
//	};
	H7_state_e state;
} H7_QEIHandler_s;

//------------------- Macros and Global Variables -------------------//


//** Macros **//


//** Global variables **//
extern H7_QEIHandler_s QEI1;
extern H7_QEIHandler_s QEI2;

//------------------- Function Declaration -------------------//
H7_state_e H7_QEI_init(H7_QEIHandler_s *h7qei, TIM_HandleTypeDef *htim);
vs32 H7_QEI_read(H7_QEIHandler_s *h7qei);


#endif /* SRC_DRIVERS_QEI_H_ */

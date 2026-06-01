/**
 * @file    EXTI.h
 * @brief   EXTI header driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup GPIO
 */

#ifndef SRC_BIOS_EXTI_H_
#define SRC_BIOS_EXTI_H_





//** Includes **//
#include "../Platform/H7_system.h"


//------------------- Enumeration -------------------//


//------------------- Structures -------------------//
typedef struct{
	u16 GPIO_EXTI_PIN;
	H7_state_e state;
} H7_EXTIHandler_s;

//------------------- Macros and Global Variables -------------------//


//** Macros **//


//** Global variables **//


//------------------- Function Declaration -------------------//
H7_state_e H7_EXTI_init(H7_EXTIHandler_s *H7EXTI, u16 GPIO_Pin, u32 PreemptPriority);


#endif /* SRC_BIOS_EXTI_H_ */

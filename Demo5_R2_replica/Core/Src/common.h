/**
 * @file    commen.h
 * @brief   User system initialization file
 * @author  Your Name
 * @date    2026-2-12
 *
 * @details All user RTOS and system initiialization, for the robots should be in this file
 *
 * @note
 */

#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_

#include "adapter.h"


//------------------- Enumeration -------------------//
//------------------- Structures -------------------//




//------------------- Macros and Global Variables -------------------//

//** Macros **//





//** FreeRTOS Global Variables and Handlers **//

// Thread Handlers
extern osThreadId_t MainTaskHandle;
extern osThreadId_t CalculationTaskHandle;

extern osThreadId_t SecondaryTaskHandle;
extern osThreadId_t align1TaskHandle;
extern osThreadId_t align2TaskHandle;
extern osThreadId_t align3TaskHandle;
extern osThreadId_t align4TaskHandle;
extern osThreadId_t Ext_TaskHandle;
extern osThreadId_t KFS_TaskHandle;
//osThreadId_t uartpcinputTaskHandle;
extern osThreadId_t NaviTaskHandle;
extern osThreadId_t MC_TaskHandle;
extern osThreadId_t MF_TaskHandle;
extern osThreadId_t SerialTaskHandle;
extern osThreadId_t fivemsTaskHandle;
extern osMessageQueueId_t uartQueue;
//osThreadId_t ModbusTaskHandle;
// Timer Callback
extern osTimerId_t CalcTimer;

// Semaphore
extern osSemaphoreId_t calcSemaphore;
extern osSemaphoreId_t fivemsSemaphore;

//** Global variables **//

extern PID_t pp_x,pp_y,pid_pres,pid_z,imu_rotate;;


//------------------- Function Declaration -------------------//

void set(void);
#endif /* SRC_COMMON_H_ */

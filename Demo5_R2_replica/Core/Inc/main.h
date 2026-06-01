/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#define STOP_CONDITION fabs(x_vel) + fabs(y_vel) + fabs(w_vel) < 0.05
#define REAR2FRONT_RATIO ((42.0f/26.0f)/(48.0f/22.0f))
typedef enum{
	CLIMBING,
	RETRACTING,
}ExtMode_t;

typedef enum {
	Ext_EXTENDED,
	Ext_RETRACTED,
	Ext_AUTO,
	Ext_MANUAL
} ExtState_t;

typedef enum{
	MANUAL,
	A_CLIMB,
	A_DESCEND,
	STOP,
	NAVI_PP,
	NAVI_MC,
	NAVI_MF,
}NavMode_t;
typedef enum{
	NAV_IDLE,
	NAV_START,
	IN_PROGRESS,
	COMPLETED
}AutoNav_t;
typedef enum{
	CLIMB,
	DESCEND,
	STOP_NAVI
}MANUAL_Navi_t;

typedef struct {
    float x;
    float y;
    float w; // Angular velocity
} vel_cmd_t;
extern vel_cmd_t vel_cmd,target_vel;

typedef enum{
	FRONT,
	BACK,
	R_SIDE,
	L_SIDE
}Direction_t;
//UART
typedef struct {
    uint8_t object_count;
    struct {
        uint16_t x;
        uint16_t y;
        uint8_t size;
    } objects[10];
} ObjectData_t;


// R2_PATH

typedef enum {
    R1_ZONE_UNKNOWN         = 0,
    R1_ZONE_MARTIAL_CLUB    = 1,
    R1_ZONE_FOREST_ENTRANCE = 2,
    R1_ZONE_FOREST          = 3,
    R1_ZONE_ARENA           = 4,
} R1_Zone_t;

typedef struct {
    float     x;     // meters
    float     y;     // meters
    R1_Zone_t zone;  // current game field zone
} R1_Status_t;

void MainTaskEntry(void *argument);
void CalculationTaskEntry(void *argument);
void SecondaryTask (void *argument);
void align(void *argument);
void align1(void *argument);
void align2(void *argument);
void align3(void *argument);
void align4(void *argument);
void Ext_Task(void *argument);
void KFS_Task(void *argument);
void SerialTask(void *argument);
void MC_Task (void *arg);
void fivems_Task(void * arg);
void navi();
float clamp(float min,float value,float max);
void velramp();
void swervepos();
void encoderpos();
void Parse3Floats(char* str);
float round_mf(float input);
void NaviTask(void* arg);
void Ext_WheelRun();
void spearHandler();
static inline float Ext_conv(float cm){
	return (cm/6.263);
}
static inline float KFS_conv(float cm){
	return (5.4 * cm)/47.5;
}


// Timer callback
//void CalcCallback(void *argument);

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

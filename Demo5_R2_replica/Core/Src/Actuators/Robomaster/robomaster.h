/*
 * Robomaster.h
 *
 *  Created on: 25 Jul 2023
 *      Author: KZ
 */

/*
 * robomaster.h
 *
 * 	modified by: Mohammed Abdulalem
 *  modified on: Nov 27, 2025
 *
 *  Notes:
 *  - Added FDCAN for communication
 *  - rbms1, rbms2 are for FDCAN1
 *  - rbms3, rbms4 are for FDCAN2
 *  - rbms5, rbms6 are for FDCAN3
 */


#ifndef SRC_ROBOMASTER_ROBOMASTER_H_
#define SRC_ROBOMASTER_ROBOMASTER_H_


//#define F1
#define F4

/***includes***/
#ifdef F1
#include "main.h"
#include "stm32f1xx_hal.h"
//#include "PID.h"
#define chlim(a, b) \
	if(a >= b)			a = b;\
	else if(a <= -b) 	a = -b;
#endif
#ifdef F4
//#include "../BIOS/system.h"
#include "../../BIOS/COM/FDCAN.h"
#include "../../Algorithms/PID/PID.h"
#endif
#include <math.h>

/***C610 DEFINES***/
// 2006 motor control current, range [-10000,10000]
#define RBMS_C610_MAX_INPUT 		10000	//rated current 3.0 A for continuous operation, here 8000 means 8.0A
#define RBMS_C610_INPUT_GRADIENT	1000
#define RBMS_C610_GEAR_RATIO		36
#define RBMS_C610_SHAFT_DIAMETER	0.006

/***C620 DEFINES***/
// 3508 motor control current, range [-16384,16384]
#define RBMS_C620_MAX_INPUT 		16384	//max current 10.0A
#define RBMS_C620_MAX_DUTY	 		819.2
#define RBMS_C620_INPUT_GRADIENT	819.2
#define RBMS_C620_GEAR_RATIO		19.2032
#define RBMS_C620_SHAFT_DIAMETER	0.01

/***GM6020 DEFINES***/
// 6020 motor control current, range [-30000,30000]
#define RBMS_GM6020_MAX_INPUT 		8192	//max current 10.0A
#define RBMS_GM6020_MAX_DUTY	 	819.2
#define RBMS_GM6020_INPUT_GRADIENT	819.2
#define RBMS_GM6020_GEAR_RATIO		19.2032
#define RBMS_GM6020_SHAFT_DIAMETER	0.01

/***WHEEL DIAMETER DEFINES***/
#define RBC_OMNI_WHEEL_DIAMETER			0.125
#define RBC_LARGE_OMNI_WHEEL_DIAMETER	0.15
//#define RBC_MECANNUM_WHEEL_DIAMETER

typedef enum {
	RBMS1 = 0,
	RBMS2,
	RBMS3,
	RBMS4,

	RBMS_ALL,
} RBMS_Num_t;

typedef enum {
	RBMS_5678 = 0x1FF,
	RBMS_1234,
} RBMS_Instance_t;

typedef enum {
	C610 = 1,
	C620,
} RBMS_Controller_t;

typedef enum {
	IDLE = 0,
	CURRENT,
	VELOCITY,
	POSITION,
} RBMS_Control_Mode_t;

typedef struct {
	struct {
		RBMS_Controller_t Controller;
		float gear_ratio;
		float wheel_diameter;
		float cur_limit;
		float vel_limit;
		float pos_limit;
		float VEL_P;
		float VEL_I;
		float VEL_D;
		float POS_P;
		float POS_I;
		float POS_D;
	} config;

	float input_gradient;
	int16_t max_input;
	float type_gear_ratio;
	uint16_t raw_pos;	//0 to 8191(unsigned)
	uint16_t prev_raw_pos;
	int16_t delta_pos;
	int cycle_cnt;
	uint16_t offset_raw_pos;
	int offset_cycle_cnt;
	int raw_rotor_pos;	//absolute angle of rotor (raw data)
	int16_t raw_rpm;
	int16_t raw_current;	//raw current data (C610: real current * 1000)
	uint8_t temperature;
	uint8_t reset_pos;
	// converted reading value
	float pos;
	float offset_pos;
	float rotor_pos;	//absolute angle of rotor (cumulative)
	float shaft_pos;
	float rpm;		//shaft rpm
	float current;	//real torque current (in Ampere)

	// target output
	float t_pos;
	float t_rotor_pos;
	float t_shaft_pos;
	float t_rpm;
	float t_current;

	PID_t P_PID, V_PID;
	float vel_error;
	float pos_error;

	RBMS_Control_Mode_t control_mode;
} rbms_motor_t;

typedef struct {
	uint8_t Init;
	uint8_t PID_Init;
	RBMS_Instance_t Instance;
	FDCAN_HandleTypeDef *hfdcan;
	FDCAN_RxHeaderTypeDef *pRxMsg;
	FDCAN_TxHeaderTypeDef pTxMsg;
	uint8_t rxdata[8];
	rbms_motor_t motor[4];	//feedback
} RBMS_t;

/***object variables***/
extern RBMS_t rbms1, rbms2, rbms3, rbms4, rbms5, rbms6;

/***prototype function***/
void RBMS_Init(RBMS_t *rbms, FDCAN_HandleTypeDef *hcan, RBMS_Instance_t instance);
void RBMS_Config(RBMS_t *rbms, RBMS_Num_t num, RBMS_Controller_t type, float gear_ratio);
void RBMS_PID_Init(RBMS_t *rbms);
void RBMS_Set_Control_Mode(RBMS_t *rbms, RBMS_Num_t num, RBMS_Control_Mode_t mode);
void RBMS_Set_Target_Current(RBMS_t *rbms, RBMS_Num_t num, float current);
void RBMS_Set_Target_Velocity(RBMS_t *rbms, RBMS_Num_t num, float rpm);
void RBMS_Set_Target_Position(RBMS_t *rbms, RBMS_Num_t num, float pos);
void RBMS_Velocity_Control(RBMS_t *rbms, RBMS_Num_t num);
void RBMS_Position_Control(RBMS_t *rbms, RBMS_Num_t num);
void RBMS_5ms(RBMS_t *rbms);
void RBMS_Send(RBMS_t *rbms);
void RBMS_ACST_Calculation(RBMS_t *rbms, RBMS_Num_t num);
void RBMS_CAN_Handler(FDCAN_RxHeaderTypeDef *pRxMsg, uint8_t rxdata[8], uint8_t can);


#endif /* SRC_ROBOMASTER_ROBOMASTER_H_ */

/*
 * Mi_Motor.h
 *
 *  Created on: Apr 30, 2024
 *      Author: yvc
 *
 *  Modified for H7 compatibility by Mohammed Abdulalem on Dec 17, 2025
 */

#ifndef SRC_CYBERGEAR_MI_MOTOR_H_
#define SRC_CYBERGEAR_MI_MOTOR_H_


/* Includes ------------------------------------------------------------------*/
//#include "../BIOS/system.h"
#include "../../BIOS/COM/FDCAN.h"

/* Definitions ---------------------------------------------------------*/

//#define Master_CAN_ID 0x00
//
//#define Communication_Type_GetID 0x00     //获取设备的ID和64位MCU唯一标识符
//#define Communication_Type_MotionControl 0x01 	//用来向主机发送控制指令
//#define Communication_Type_MotorRequest 0x02	//用来向主机反馈电机运行状态
//#define Communication_Type_MotorEnable 0x03	//电机使能运行
//#define Communication_Type_MotorStop 0x04	//电机停止运行
//#define Communication_Type_SetPosZero 0x06	//设置电机机械零位
//#define Communication_Type_CanID 0x07	//更改当前电机CAN_ID
//#define Communication_Type_Control_Mode 0x12
//#define Communication_Type_GetSingleParameter 0x11	//读取单个参数
//#define Communication_Type_SetSingleParameter 0x12	//设定单个参数
//#define Communication_Type_ErrorFeedback 0x15	//故障反馈帧֡
//
//#define Run_mode 0x7005
//#define Iq_Ref 0x7006
//#define Spd_Ref 0x700A
//#define Limit_Torque 0x700B
//#define Cur_Kp 0x7010
//#define Cur_Ki 0x7011
//#define Cur_Filt_Gain 0x7014
//#define Loc_Ref 0x7016
//#define Limit_Spd 0x7017
//#define Limit_Cur 0x7018
//
//#define Gain_Angle 720/32767.0
//#define Bias_Angle 0x8000
//#define Gain_Speed 30/32767.0
//#define Bias_Speed 0x8000
//#define Gain_Torque 12/32767.0
//#define Bias_Torque 0x8000
//#define Temp_Gain 0.1
//
//#define MAX_P 720
//#define MIN_P -720
//#define MAX_S 30
//#define MIN_S -30
//#define MAX_T 12
//#define MIN_T -12
//
//#define Motor_Error 0x00
//#define Motor_OK 0X01
//
//#define Current_mode 3
//#define Position_mode 1
//#define Motion_mode 0
//#define Speed_mode 2
//
//typedef struct{
//	CAN_HandleTypeDef *hcan;
//	uint8_t CAN_ID;
//	int Motor_Nbr;
//
//	float Angle;
//	float Speed;
//	float Torque;
//	float Temp;
//
//	uint16_t set_current;
//	uint16_t set_speed;
//	uint16_t set_position;
//
//	uint8_t error_code;
//
//	float Angle_Bias;
//
//}MI_Motor;
//
//void Init_Motor(MI_Motor *Motor,uint8_t Can_Id,int Motor_Num,float mode);
//void Enable_Motor(MI_Motor *Motor);
//void Motor_Data_Handler(MI_Motor *Motor,uint8_t DataFrame[8],uint32_t IDFrame);
//uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame);
//float uint16_to_float(uint16_t x,float x_min,float x_max,int bits);
//void Set_Mode(MI_Motor *Motor,float Mode);
//void Set_Current(MI_Motor *Motor,float Current);
//void Set_Motor_Parameter(MI_Motor *Motor,uint16_t Index,float Value,char Value_type);
//void Set_ZeroPos(MI_Motor *Motor);
//void Set_CANID(MI_Motor *Motor,uint8_t CAN_ID);

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f
typedef enum
{
	OK                 = 0,//无故障
	BAT_LOW_ERR        = 1,//欠压故障
	OVER_CURRENT_ERR   = 2,//过流
	OVER_TEMP_ERR      = 3,//过温
	MAGNETIC_ERR       = 4,//磁编码故障
	HALL_ERR_ERR       = 5,//HALL编码故障
	NO_CALIBRATION_ERR = 6//未标定
} motor_state_e;//电机状态（故障信息）

typedef enum
{
	RESET_MODE = 0,//Reset[模式]
	CALI_MODE  = 1,//Cali 模式[标定]
	RUN_MODE   = 2//Motor模式[运行]
} motor_mode_e;//电机运行模式

typedef struct
{
	uint32_t motor_id : 8; // 只占8位
	uint32_t data : 16;
	uint32_t mode : 5;
	uint32_t res : 3;
} EXT_ID_t; // 32位扩展ID解析结构体

typedef struct
{
	// 电机反馈
	int16_t angle_temp;
	int16_t speed_temp;
	int16_t torque_temp;
	int16_t temprature_temp;

	float angle; // 连续角
	float speed;
	float torque;
	float temprature;
	uint32_t last_update_time; // 编码器时间戳
} Motor_fdb_t;                 // 电机编码器反馈结构体

typedef struct
{
	FDCAN_HandleTypeDef *phfdcan;
	motor_state_e motor_state;
	motor_mode_e  motor_mode;
	EXT_ID_t EXT_ID;
	uint8_t txdata[8];
	Motor_fdb_t motor_fdb;
} MI_Motor_t;

/**********************Functions*************************8*/
void MI_motor_get_ID(MI_Motor_t* hmotor);
void MI_motor_init(MI_Motor_t* hmotor,FDCAN_HandleTypeDef *phfdcan);
void MI_motor_enable(MI_Motor_t *hmotor, uint8_t id);
void MI_motor_controlmode(MI_Motor_t* hmotor, float torque, float MechPosition , float speed , float kp , float kd);
void MI_motor_stop(MI_Motor_t *hmotor);
void MI_motor_setMechPosition2Zero(MI_Motor_t *hmotor);
void MI_motor_changeID(MI_Motor_t* hmotor,uint8_t Now_ID,uint8_t Target_ID);
void MIMotor_MotorDataDecode(uint32_t rx_EXT_id, uint8_t rxdata[]);
void MI_Motor_FDCanTx(MI_Motor_t* hmotor);

extern MI_Motor_t MI_Motor;

#endif /* SRC_CYBERGEAR_MI_MOTOR_H_ */

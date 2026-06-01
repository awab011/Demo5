/**
 * @file    commen.c
 * @brief   User system initialization file
 * @author  Your Name
 * @date    2026-2-12
 *
 * @details All user RTOS and system initiialization, for the robots should be in this file
 *
 * @note
 */


#include "common.h"

//------------------- Macros and Global Variables -------------------//

//** Macros **//


//** FreeRTOS Global Variables and Handlers **//

// Thread Handlers
osThreadId_t MainTaskHandle;
osThreadId_t CalculationTaskHandle;
osThreadId_t SecondaryTaskHandle;
osThreadId_t align1TaskHandle;
osThreadId_t align2TaskHandle;
osThreadId_t align3TaskHandle;
osThreadId_t align4TaskHandle;
osThreadId_t Ext_TaskHandle;
osThreadId_t KFS_TaskHandle;
//osThreadId_t uartpcinputTaskHandle;
osThreadId_t NaviTaskHandle;
osThreadId_t MC_TaskHandle;
osThreadId_t MF_TaskHandle;
osThreadId_t SerialTaskHandle;
osThreadId_t fivemsTaskHandle;
osMessageQueueId_t uartQueue;
//osThreadId_t ModbusTaskHandle;



// Timer Callback
osTimerId_t CalcTimer;


// Semaphore
osSemaphoreId_t calcSemaphore;
osSemaphoreId_t fivemsSemaphore;


//** Global variables **//

PID_t pp_x,pp_y,pid_pres,pid_z,imu_rotate;

//------------------- Function Definition -------------------//

void set(void){


	initialize();

	// User costume initializaton

	TF_Init_I2C_Async(&TF_F,&hi2c5,0x13U,TF_I2C_IT);
	TF_Init_I2C_Async(&TF_B,&hi2c5,0x12U,TF_I2C_IT);
//	TF_Init_I2C_Async(&TF_L,&hi2c2,0x11U,TF_I2C_IT);
	TF_Init_I2C_Async(&TF_R,&hi2c5,0x14U,TF_I2C_IT);
	TF_Bus_Register(&i2c5_bus,&TF_B,&TF_B_data);
	TF_Bus_Register(&i2c5_bus,&TF_F,&TF_F_data);
	TF_Bus_Register(&i2c5_bus,&TF_R,&TF_R_data);


	// Robomaster


	RBMS_Init(&rbms1,&hfdcan1,RBMS_1234);
	RBMS_Init(&rbms2,&hfdcan1,RBMS_5678);
	RBMS_Init(&rbms3,&hfdcan2,RBMS_1234);
	RBMS_Init(&rbms4,&hfdcan2,RBMS_5678);




	//Swerve
	RBMS_Config(&rbms1, RBMS1, C610, 2.0f);
	RBMS_Config(&rbms1, RBMS2, C610, 2.0f);
	RBMS_Config(&rbms1, RBMS3, C610, 2.0f);
	RBMS_Config(&rbms1, RBMS4, C610, 2.0f);


// 	//Extensions
 	RBMS_Config(&rbms2, RBMS1, C620, 1.0f); //FExt_M
 	RBMS_Config(&rbms2, RBMS2, C620, 1.0f); //BExt_M
 	RBMS_Config(&rbms2, RBMS3, C610, 1.0f); //BExt_RW
 	RBMS_Config(&rbms2, RBMS4, C610, 1.0f); //BExt_LW
 	RBMS_Config(&rbms3, RBMS1, C610, 1.0f); //FExt_RW
 	RBMS_Config(&rbms3, RBMS2, C610, 1.0f); //FExt_LW


 	//SpearArm
 	RBMS_Config(&rbms3, RBMS3, C610, 1.0f); //SpearArm
 	RBMS_Config(&rbms3, RBMS4, C610, 1.0f); //SpearGripper

 	//KFS
 	RBMS_Config(&rbms4, RBMS1, C610, 1.0f); //KFS_R
 	RBMS_Config(&rbms4, RBMS2, C620, 1.0f); //KFS_YAW
 	RBMS_Config(&rbms4, RBMS3, C620, 1.0f); //KFS_G
 	RBMS_Config(&rbms4, RBMS4, C620, 1.0f); //KFS_Z



	rbms1.motor[RBMS1].config.vel_limit = 300;
	rbms1.motor[RBMS2].config.vel_limit = 300;
	rbms1.motor[RBMS3].config.vel_limit = 300;
	rbms1.motor[RBMS4].config.vel_limit = 300;

	rbms1.motor[RBMS4].config.POS_P = 300;
	rbms1.motor[RBMS3].config.POS_P = 300;
	rbms1.motor[RBMS2].config.POS_P = 300;
	rbms1.motor[RBMS1].config.POS_P = 300;

 	rbms4.motor[RBMS2].config.POS_P = 300;
 	rbms4.motor[RBMS2].config.vel_limit = 100;
 	rbms4.motor[RBMS2].config.cur_limit = 15;

 	rbms4.motor[RBMS4].config.POS_P = 300;


 	rbms3.motor[RBMS4].config.POS_P = 300;
 	rbms4.motor[RBMS3].config.POS_P = 300;
 	rbms4.motor[RBMS4].config.POS_P = 200;


	RBMS_PID_Init(&rbms1);
 	RBMS_PID_Init(&rbms2);
 	RBMS_PID_Init(&rbms3);
 	RBMS_PID_Init(&rbms4);

	RBMS_Set_Control_Mode(&rbms1, RBMS1, POSITION);
	RBMS_Set_Control_Mode(&rbms1, RBMS2, POSITION);
	RBMS_Set_Control_Mode(&rbms1, RBMS3, POSITION);
	RBMS_Set_Control_Mode(&rbms1, RBMS4, POSITION);

 	RBMS_Set_Control_Mode(&rbms2, RBMS1, POSITION);
 	RBMS_Set_Control_Mode(&rbms2, RBMS2, POSITION);
 	RBMS_Set_Control_Mode(&rbms2, RBMS3, VELOCITY);
 	RBMS_Set_Control_Mode(&rbms2, RBMS4, VELOCITY);
 	RBMS_Set_Control_Mode(&rbms3, RBMS1, VELOCITY);
 	RBMS_Set_Control_Mode(&rbms3, RBMS2, VELOCITY);

 	RBMS_Set_Control_Mode(&rbms3, RBMS3, VELOCITY);
 	RBMS_Set_Control_Mode(&rbms3, RBMS4, POSITION);

 	RBMS_Set_Control_Mode(&rbms4, RBMS1, VELOCITY);
 	RBMS_Set_Control_Mode(&rbms4, RBMS2, POSITION);
 	RBMS_Set_Control_Mode(&rbms4, RBMS3, POSITION);
 	RBMS_Set_Control_Mode(&rbms4, RBMS4, POSITION);

	// Navi logic
	MODNRobotBaseVelInit(MODN_FWD_SWERVE, 0.583, 0.353, &modn);
	MODNRobotConInit(&x_vel, &y_vel, &w_vel, &modn);
	MODNWheelVelInit(&myswerve.vel[0],&myswerve.vel[1],&myswerve.vel[2],&myswerve.vel[3],&modn);
	MODNWheelDirInit(&myswerve.ang[0],&myswerve.ang[1],&myswerve.ang[2],&myswerve.ang[3],&modn);

	SwerveInit(MODN_FWD_SWERVE,2.0f,0.583, 0.353,&vesc,&rbms1,&myswerve);

	VESCInit(101,102,103,104,&vesc);
	vesc.pole_pairs=28;
	vesc.gear_ratio=1;
	vesc.wheel_diameter=0.0853;

}

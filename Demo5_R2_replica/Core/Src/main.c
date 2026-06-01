/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @version		: H7Lib_1.0
  ******************************************************************************
  * @attention
  *
  */

//** Includes **//
#include "main.h"
#include "Platform/H7_system.h"
#include "adapter.h"
#include "common.h"
#include "r2_world_state.h"
#include "r2_subsystems.h"
#include <errno.h>

#define J60_ID 1
//-6.61099577 = 41.402 = 6.2623

// 6.60471535 = 41.37  = 6.2637
//7.01981258 SPEAR_ARM
//5.34070969 KFS_Z
/**
 * Block 1: available is NONE
 * Block 2: available is F,L,R
 * Block 3: available is F
 * Block 4: available is F,L,B
 * Block 5: available is F,L
 * Block 6: available is NONE
 * Block 7: available is L
 * Block 8: available is NONE
 * Block 9: available is B,R
 * Block 10: available is L,B
 * Block 11: available is B
 * Block 12: available is B,R
 * */
float FilteredTF[4] = {0};
vel_cmd_t vel_cmd,target_vel;

#define TF_FRONT_I2C_ADDR 0x13U
#define TF_BACK_I2C_ADDR  0x12U
#define TF_LEFT_I2C_ADDR  0x11U
#define TF_RIGHT_I2C_ADDR 0x14U

#define FOREST_DIST 0.5

/*----------Forest Sensors Registers----------*/
/* All active high */
#define F_FWRD   &GPIOB->IDR,15	 //IP6
#define UP_F 	 &GPIOC->IDR,8	 //IP9
#define DF_20 	 &GPIOD->IDR,10 	 //IP7
#define UP_B	 &GPIOD->IDR,3 	 //IP11
#define DOWN_B 	 &GPIOD->IDR,7  	 //IP13
#define DF_40 	 &GPIOC->IDR,8
#define SPEAR_IR &GPIOB->IDR,6
#define KFS_IR	 &GPIOB->IDR,5

//IP5, IP13, IP15
/*--------Robomaster Limits---------*/
#define SPEAR_ARM_MAX 7.0
#define PICKER_270	0.74
#define PICKER_120	0.34
#define PICKER_220	0.61

//1.50705016
//0.739502072

#define KFS_Z_MAX 5.35
#define KFS_Z_20  2.15
#define KFS_Z_40  4.6
#define KFS_Z_0	  0.0
#define KFS_Z_S2  4.7


#define KFS_RACK_M 5.15
#define KFS_RACK_T 5.15

#define KFS_G_V      0.45
#define KFS_G_Grip   0.45
#define KFS_G_Tight	 0.51
#define KFS_G_MAX    0.0
#define KFS_G_MAX1   0.13

#define KFS_YAW_180 -3.0
#define KFS_YAW_100 -1.75
#define KFS_YAW_90  -2.0
#define KFS_YAW_0	-0.2
//#define KFS_YAW_0	0.0

#define YAW_V		1.5
#define YAW_AD		0.4
#define YAW_DD		0.4

#define KFS_Z_STORE_1 0.0
#define KFS_Z_STORE_2 2.8

#define EXT_RACK_M Ext_conv(46.0)
#define EXT_RACK_T Ext_conv(42.0)

/*------Robomaster Definitions------*/
/* Can 1*/
//Swerve

// Extensions
#define FExt_M  &rbms2,RBMS1 // Down is POSITIVE
#define BExt_M  &rbms2,RBMS2 // Down is NEGATIVE
#define BExt_RW &rbms2,RBMS3 //	Front is POSITIVE
#define BExt_LW &rbms2,RBMS4 // Front is NEGATIVE

/* Can 2*/

//Extensions
#define FExt_RW &rbms3,RBMS1 //Front is POSTIVE
#define FExt_LW &rbms3,RBMS2 //Front is NEGATIVE


//Spearhead
#define SpearArm     &rbms3,RBMS3 //Back(Right) is POSITIVE
#define SpearGripper &rbms3,RBMS4 //Anti-Clockwise is POSITIVE (Rotation to the front)

//KFS
#define KFS_R   &rbms4,RBMS1 // KFS Right
//#define KFS_L &rbms4,RBMS2 // KFS Left
#define KFS_YAW &rbms4,RBMS2 // KFS Rotation
#define KFS_G   &rbms4,RBMS3 // KFS Gripper
#define KFS_Z   &rbms4,RBMS4 // KFS Z Axis
#define KFS_L_ID 115

/*------Extension Position Definitions------*/

#define FRONT_EXT_POS	 &rbms2.motor[RBMS1].pos
#define REAR_EXT_POS 	 &rbms2.motor[RBMS2].pos
#define SPEAR_ARM_POS	 &rbms3.motor[RBMS3].pos
#define SPEAR_PICKER_POS &rbms3.motor[RBMS4].pos
#define KFS_G_POS 	 	 &rbms4.motor[RBMS3].pos
#define KFS_Z_POS 		 &rbms4.motor[RBMS4].pos
#define KFS_YAW_POS 	 &rbms4.motor[RBMS2].pos


/** ============================================================
 * Received Data From CAN:
 *   0x105 -> Forest blocks 1-8
 *   0x106 -> Forest blocks 9-12 + status (lidar/camera/end)
 *   0x107 -> R1 status (x, y, zone)
 *   0x108 -> R2 pose (x, y, z, yaw)
 *
 * R1 Zone mapping:
 *   0 = Unknown
 *   1 = Martial Club
 *   2 = Forest Entrance  (R1 exiting, wait for clear!)
 *   3 = Forest           (climbing area cleared)
 *   4 = Arena
 * ============================================================ */
/* Laptop-derived state. See r2_world_state.h for the contract:
 *   g_raw   - written by FDCAN3 ISR (callbacks.c), volatile.
 *   g_world - written by fivems_Task, read by app tasks. */
volatile R2RawState_t g_raw   = {0};
R2WorldState_t        g_world = {0};
uint8_t s_pick_flag = 0;
float mc_kp, mc_ki, mc_kd;


/*---------------------------Entry Blocks Points---------------------------------*/
/* Type definitions for these state machines live in r2_subsystems.h. */

/*X is forward and Y is to the left - Refer to REP 103*/
const PP_Points_t entry_point[3] = {
		{.x = 1.0, .y = 2.0},
		{.x = 2.0, .y = 3.0},
		{.x = 3.0, .y = 4.0},
};

PP_Points_t spear_point[5] = {
		{.x = 0.41, .y = 0.23, .z = NAN},
		{.x = 0.6,  .y = 0.23, .z = NAN},
		{.x = 0.77, .y = 0.23, .z = NAN},
		{.x = 0.94, .y = 0.23, .z = NAN},
		{.x = 1.1,  .y = 0.23, .z = NAN}
};

/* MC subsystem instance */
MC_Handle_t MC = {.cmd = MC_CMD_IDLE, .timer = 0.0, .assembled = false, .pick_state = MC_PICK_IDLE, .target_point = 1};
/* KFS subsystem instance */
KFS_Handle_t KFS = {.place_state = PLACE_IDLE, .pick_state = PICK_IDLE,
		.cmd = KFS_CMD_IDLE};

/* Ext subsystem instance */
Ext_Handle_t Ext = {.climb_state = CLIMB_IDLE, .descend_state = DESCEND_IDLE,
		.cmd = EXT_CMD_IDLE};

/* MF (Meihua-Forest) alignment instance.
 *
 * Available walls per block (1-indexed):
 *   B1: none, B2: N|W|E, B3: N, B4: N|W|S, B5: N|W, B6: none,
 *   B7: W,    B8: none,  B9: S|E, B10: W|S, B11: S, B12: S|E.
 */
MF_Align_t MF_Align = {.dist_x = NULL, .dist_y = NULL, .heading = HEADING_NORTH, .dir_x = 0, .dir_y = 0, .x_aligned = false, .y_aligned = false};


//1-Indexed
static const uint8_t block_walls[13] = {
		0,                          // block 0 {outside the forest)
		0,                          // block 1
		DIR_N | DIR_W | DIR_E,      // block 2
		DIR_N,                      // block 3
		DIR_N | DIR_W | DIR_S,      // block 4
		DIR_N | DIR_W,              // block 5
		0,                          // block 6
		DIR_W,                      // block 7
		0,                          // block 8
		DIR_S | DIR_E,              // block 9
		DIR_W | DIR_S,              // block 10
		DIR_S,                      // block 11
		DIR_S | DIR_E,              // block 12
};

/* I think this is faster than using a rotation*/
//sensor_dir[HEADING][SENSOR_IDX]
static const uint8_t sensor_dir[4][4] = { // The arrangement is F,L,B,R
		{DIR_N, DIR_W, DIR_S, DIR_E}, //Heading FRONT tangle is 0, robot is facing NORTH
		{DIR_W, DIR_S, DIR_E, DIR_N}, //Heading LEFT  tangle is 90, robot is facing WEST
		{DIR_S, DIR_E, DIR_N, DIR_W}, //Heading BACK  tangle is 180 or -180, robot is facing SOUTH
		{DIR_E, DIR_N, DIR_W, DIR_S}  //Heading RIGHT tangle is -90, robot is facing EAST
};

int getSensorPointingTo(Heading_t robot_heading, uint8_t target_world_dir) {
	for (int i = 0; i < 4; i++) {
		if (sensor_dir[robot_heading][i] == target_world_dir) {
			return i;
		}
	}
	return -1;
}


void alignToWorld(uint8_t current_block, Heading_t heading) {
	uint8_t available_walls = block_walls[current_block];

	if (!available_walls) return;
	if (heading == -1) return;

	if (available_walls & DIR_N) {
		int sensor_idx = getSensorPointingTo(heading, DIR_N);
		MF_Align.dist_x = &FilteredTF[sensor_idx];
		MF_Align.dir_x  = 1;
	}
	else if (available_walls & DIR_S) {
		int sensor_idx = getSensorPointingTo(heading, DIR_S);
		MF_Align.dist_x = &FilteredTF[sensor_idx];
		MF_Align.dir_x = -1;

	}
	else{
		MF_Align.dir_x = 0;
		MF_Align.dist_x = NULL;
	}

	if (available_walls & DIR_W) {
		int sensor_idx = getSensorPointingTo(heading, DIR_W);
		MF_Align.dist_y = &FilteredTF[sensor_idx];
		MF_Align.dir_y = 1;
	}
	else if (available_walls & DIR_E) {
		int sensor_idx = getSensorPointingTo(heading, DIR_E);
		MF_Align.dist_y = &FilteredTF[sensor_idx];
		MF_Align.dir_y = -1;
	}
	else{
		MF_Align.dist_y = NULL;
		MF_Align.dir_y = 0;
	}
}
/* PathApp_t kept here because R2CAN_Path comes from can_packet.h, not in r2_subsystems.h. */
typedef struct {
	int          steps_done;
	int          current_step;
	AppStatus_t  step_status;
	AppCmd_t     cmd;
	R2CAN_Path*  Path;
} PathApp_t;
PathApp_t PathApp = {.steps_done = 0, .step_status = APP_IDLE, .current_step = 0, .cmd = APP_CMD_IDLE, .Path = &path};
void ApplySteps(PathApp_t* PathApp)
{

	switch(PathApp->step_status)
	{
	case APP_IDLE:
		break;
	case APP_Start:
		Heading_t heading = -1;
		switch(PathApp->Path->steps[PathApp->current_step].height_delta_enc)
		{
		case R2CAN_HDELTA_POS400:
			Ext.level = EXT_LEVEL_40;
			KFS.level = KFS_LEVEL_40;
			break;
		case R2CAN_HDELTA_NEG400:
			Ext.level = EXT_LEVEL_40;
			KFS.level = KFS_LEVEL_40;
			break;
		case R2CAN_HDELTA_POS200:
			Ext.level = EXT_LEVEL_20;
			KFS.level = KFS_LEVEL_20;
			break;
		case R2CAN_HDELTA_NEG200:
			Ext.level = EXT_LEVEL_20;
			KFS.level = KFS_LEVEL_20;
			break;
		default:
			Ext.level = EXT_LEVEL_INVALID;
			KFS.level = KFS_LEVEL_INVALID;
			break;
		}
		switch(PathApp->Path->steps[PathApp->current_step].direction)
		{
		case R2CAN_DIR_DOWN:
			Ext.direction = EXT_DIR_FORWARD;
			KFS.direction = EXT_DIR_FORWARD;
			heading = HEADING_NORTH;
			break;
		case R2CAN_DIR_UP:
			Ext.direction = EXT_DIR_BACK;
			KFS.direction = EXT_DIR_BACK;
			heading = HEADING_SOUTH;
			break;
		case R2CAN_DIR_RIGHT:
			Ext.direction = EXT_DIR_LEFT;
			KFS.direction = EXT_DIR_LEFT;
			heading = HEADING_WEST;
			break;
		case R2CAN_DIR_LEFT:
			Ext.direction = EXT_DIR_RIGHT;
			KFS.direction = EXT_DIR_RIGHT;
			heading = HEADING_EAST;
			break;
		default:
			Ext.direction = EXT_DIR_INVALID;
			KFS.direction = EXT_DIR_INVALID;
			heading = -1;
			break;
		}

		MF_Align.x_aligned = false;
		MF_Align.y_aligned = false;
		alignToWorld(PathApp->Path->steps[PathApp->current_step].current_block, heading);
		PathApp->step_status = APP_ALIGN;
		break;
		case APP_PP:
			if(!mc_pp.is_moving && KFS.cmd == KFS_CMD_IDLE)
			{
				PathApp->step_status = APP_BUSY;
				if(KFS.cmd == KFS_CMD_IDLE){
					KFS.cmd = KFS_CMD_PICK;
				}
			}
			break;
		case APP_ALIGN:
			if(MF_Align.dist_x == NULL && MF_Align.dist_y == NULL) PathApp->step_status = APP_COMMAND;
			if(MF_Align.x_aligned && MF_Align.y_aligned)		   PathApp->step_status = APP_COMMAND;
			float vel_x = 0;
			float vel_y = 0;
			if(fabs(FOREST_DIST - *MF_Align.dist_x) > 0.03 )
			{
				uint8_t err_dir_x = 0;
				if(FOREST_DIST - *MF_Align.dist_x > 0) err_dir_x = -1;
				else err_dir_x = 1;
				vel_x = 0.15 * err_dir_x * MF_Align.dir_x;
			}
			else{
				MF_Align.x_aligned = true;
				vel_x = 0;
			}
			if(fabs(FOREST_DIST - *MF_Align.dist_y) > 0.03 )
			{
				uint8_t err_dir_y = 0;
				if(FOREST_DIST - *MF_Align.dist_y > 0) err_dir_y = -1;
				else err_dir_y = 1;
				vel_y = 0.15 * err_dir_y * MF_Align.dir_y;
			}
			else{
				MF_Align.y_aligned = true;
				vel_y = 0;
			}
			tVx=   vel_x*cos(IMU.real_zrad ) + vel_y*sin(IMU.real_zrad);
			tVy=  -vel_x*sin(IMU.real_zrad ) + vel_y*cos(IMU.real_zrad);
			break;
		case APP_COMMAND:
			switch(PathApp->Path->steps[PathApp->current_step].action_type)
			{
			case R2CAN_ACT_PRE_ENTRY:
				/* Path plan to target block*/
				//			Navigator_MoveTo(&mc_pp, entry_point[PathApp->Path->steps[PathApp->current_step].target_block].x,
				//					entry_point[PathApp->Path->steps[PathApp->current_step].target_block].y, NAN);
				//			PathApp->step_status = APP_PP;
				PathApp->step_status = APP_DONE;
				break;
			case R2CAN_ACT_MOVE:
				/* Set the relevant Ext_Level / KFS_Level to high_delta_enc*/
				/* if autopickup is enabled then ROTATE First then set the KFS_Cmd*/
				/* if autopickup is not enabled then straight up rotate and go forward*/
				//			if(PathApp->Path->steps[PathApp->current_step].auto_pickup)
				//			{
				//				if(KFS.cmd == KFS_CMD_IDLE){
				//					KFS.cmd = KFS_CMD_AUTOPICKUP;
				//					KFS.increment = true;
				//					Ext.increment = false;
				//				}
				//			}
				//			else{
				//			}
				Ext.cmd = EXT_CMD_FORWARD;
				Ext.increment = true;
				KFS.increment = false;
				PathApp->step_status = APP_BUSY;
				break;
			case R2CAN_ACT_PICKUP:
				/* This should just be pickup without climbing or descending, so just rotate and start the KFS state machine*/
//							if(KFS.cmd == KFS_CMD_IDLE)
//							{
//								PathApp->step_status = APP_BUSY;
//								KFS.increment = true;
//								Ext.increment = false;
//								KFS.cmd = KFS_CMD_PICK;
//							}
				PathApp->step_status = APP_DONE;
				break;
			}
			break;
			case APP_BUSY:
				break;
			case APP_DONE:
				if(PathApp->current_step < PathApp->Path->step_count)
				{
					PathApp->current_step++;
				}
				PathApp->steps_done++;
				if(PathApp->steps_done < PathApp->Path->step_count){
					PathApp->step_status = APP_Start;
				}
				break;
	}

	//	switch(PathApp->cmd)
	//	{
	//	case APP_CMD_IDLE:
	//		break;
	//	case APP_CMD_Start:
	//		break;
	//	case APP_CMD_BUSY:
	//		break;
	//	default:
	//		break;
	//	}
}

/* ------------------------------ ROBOT STATE MACHINE HANDLER ------------------------------*/
//typedef enum{
//	Robo_IDLE,
//	Robo_MC_BUSY,
//	Robo_MC_DONE,
//	Robo_MC_TO_MF,
//	Robo_MF_BUSY,
//	Robo_MF_DONE,
//	Robo_ARENA_BUSY,
//	Robo_ERR
//}Robot_state_t;
//


//typedef enum{
//
//}Robot_Handle_t;
//
//






float front_cmd , rear_cmd, sarm_cmd, sgripper_cmd, kfs_z_cmd, kfs_g_cmd, kfs_r_cmd,kfs_yaw_cmd = {0.0};
uint8_t FExt_state = 0;
uint8_t BExt_state = 0;
volatile uint32_t tim13_cnt;
//Debug Variables
uint8_t Hall[11] = {0};
// Variable definitions
double timer = 0;
double timer3 = 0;
float KFS_L_cmd = 0;
float KFS_R_cmd = 0;
float Front_TF, Back_TF, Left_TF,Right_TF;
float current_pos_j60 = 0;
// Navigation variables
NavMode_t navi_state = MANUAL;
int spear_dir = 0;
//uint8_t s_pick_flag = 0;
float mc_kp_y, mc_ki_y, mc_kd_y,mc_kn_y;
float mc_kp_x, mc_ki_x, mc_kd_x,mc_kn_x;
float mc_vel,mc_accel_d,mc_decel_d;

//path plan BlueBee
volatile uint8_t msg_received = 0;
volatile uint8_t tx_busy = 0;
uint8_t rxChar;
char rxBuffer[100];
//------------------- Macros and Global Variables -------------------//
//** Macros **//


//** Global variables **//

/**
  * @brief  The application entry point.
  * @retval int
  */
int mm;
int main(void)
{
// System Core initialization
	H7_system_init();

// Peripherals and ports initialization
	set();

  /* Init scheduler */
  osKernelInitialize();

  /* Definitions for MainTask */

  const osThreadAttr_t MainTask_attributes = {
    .name = "MainTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

  const osThreadAttr_t SecondaryTask_attributes =
  { .name = "SecondTask", .stack_size = 512 * 2, .priority =
		  (osPriority_t) osPriorityNormal, };
  /* Definitions for CalculationTask */

  const osThreadAttr_t CalculationTask_attributes = {
    .name = "CalculationTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

	const osThreadAttr_t align1Task_attributes =
	{ .name = "align1", .stack_size = 256 * 2,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t align2Task_attributes =
	{ .name = "align2", .stack_size = 256 * 2,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t align3Task_attributes =
	{ .name = "align3", .stack_size = 256 * 2,
			.priority = (osPriority_t) osPriorityNormal, };

	const osThreadAttr_t align4Task_attributes =
	{ .name = "align4", .stack_size = 256 * 2,
			.priority = (osPriority_t) osPriorityNormal, };
	const osThreadAttr_t ExtTask_attributes =
	{ .name = "Ext_Task", .stack_size = 256 * 6,
			.priority = (osPriority_t) osPriorityNormal, };
	const osThreadAttr_t KFSTask_attributes =
	{ .name = "KFS_Task", .stack_size = 256 * 6,
			.priority = (osPriority_t) osPriorityNormal, };
	//	const osThreadAttr_t NaviTask_attributes =
	//	{ .name = "Navi_Task", .stack_size = 512 * 4,
	//			.priority = (osPriority_t) osPriorityNormal, };
	const osThreadAttr_t MC_Task_attributes =
	{ .name = "MC_Task", .stack_size = 512 * 4,
			.priority = (osPriority_t) osPriorityNormal, };
	const osThreadAttr_t SerialTask_attributes =
	{ .name = "SerialTask", .stack_size = 512 * 4,
			.priority = (osPriority_t) osPriorityNormal, };
	const osThreadAttr_t fivemsTask_attributes =
	{ .name = "fivems_Task", .stack_size = 512 * 4,
			.priority = (osPriority_t) osPriorityHigh, };

  // Semaphore
  const osSemaphoreAttr_t calcSemaphore_attributes = { .name = "calcSemaphore" };
  const osSemaphoreAttr_t fivemsSemaphore_attributes = { .name = "fivemsSemaphore" };
  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle 		= osThreadNew(MainTaskEntry, NULL, &MainTask_attributes);
  /* creation of CalculationTask */
  CalculationTaskHandle = osThreadNew(CalculationTaskEntry, NULL, &CalculationTask_attributes);
  SecondaryTaskHandle 	= osThreadNew(SecondaryTask, NULL, &SecondaryTask_attributes);
  align1TaskHandle		=osThreadNew(align1, NULL, &align1Task_attributes);
  align2TaskHandle		=osThreadNew(align2, NULL, &align2Task_attributes);
  align3TaskHandle		=osThreadNew(align3, NULL, &align3Task_attributes);
  align4TaskHandle		=osThreadNew(align4, NULL, &align4Task_attributes);
  Ext_TaskHandle		=osThreadNew(Ext_Task, NULL, &ExtTask_attributes);
  KFS_TaskHandle		=osThreadNew(KFS_Task, NULL, &KFSTask_attributes);
  MC_TaskHandle			=osThreadNew(MC_Task, NULL, &MC_Task_attributes);
  SerialTaskHandle		=osThreadNew(SerialTask, NULL, &SerialTask_attributes);
  fivemsTaskHandle		=osThreadNew(fivems_Task,NULL,&fivemsTask_attributes);
  //	NaviTaskHandle=osThreadNew(NaviTask, NULL, &NaviTask_attributes);


  /* Creating Calculation semaphore for 5ms operations */
  calcSemaphore = osSemaphoreNew(1, 1, &calcSemaphore_attributes);
  fivemsSemaphore = osSemaphoreNew(1, 0, &fivemsSemaphore_attributes);
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {

  }

}

static void UART_SendString_IT_blocking_safe(const char *s)
{
	size_t len = strlen(s);

	/* wait if another TX is in flight */
	while (tx_busy) osDelay(1);

	/* mark busy then start TX */
	tx_busy = 1;
	HAL_UART_Transmit_IT(&huart5, (uint8_t *)s, len);
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
	osSemaphoreRelease(calcSemaphore);
	HAL_TIM_IRQHandler(&htim7);
}


/* USER CODE END Header_MainTaskEntry */

void MainTaskEntry(void *argument)
{
	// Make sure somewhere in your code you reference it, e.g.:
mm++;
	tangle = IMU.real_z;
	//	RBMS_Set_Control_Mode(FExt_M,      IDLE);
	RBMS_Set_Control_Mode(FExt_RW,	   VELOCITY);
	RBMS_Set_Control_Mode(FExt_LW,     VELOCITY);
	//	RBMS_Set_Control_Mode(BExt_M,      IDLE);
	RBMS_Set_Control_Mode(BExt_RW,     VELOCITY);
	RBMS_Set_Control_Mode(BExt_LW,     VELOCITY);
	//			RBMS_Set_Control_Mode(KFS_Z,       IDLE);
	//			RBMS_Set_Control_Mode(KFS_G,       IDLE);
	//	RBMS_Set_Control_Mode(SpearArm,	   IDLE);
	//			RBMS_Set_Control_Mode(KFS_YAW, IDLE);
	//	RBMS_Set_Control_Mode(SpearGripper,IDLE);


	//	RBMS_Set_Target_Position(KFS_T,0.3);
	//	uint8_t kfs_up_flag = 0;
	//	uint8_t kfs_yaw_flag = 0;
	//	uint8_t kfs_g_flag = 0;
	//	uint8_t kfs_w_flag = 0;
	//	uint8_t ext_up_flag = 0;
	uint8_t s_arm_flag = 0;
	uint8_t intake_f = 0;
	//	AK_Config(&cubemars1, AK45_36, &hcan2, 111, 1.0);
	//	osDelay(20);
	//	AK_Set_Mode(&cubemars1, AK_MODE_SERVO);
	//	osDelay(20);
	//	AK_Servo_Set_Origin(&cubemars1, 0);
	//	osDelay(20);
	//	float cubemarspos = 0.0;
	WTIMU63_SetOffset(&IMU63, 90);
	tangle = 90;
  while(1)
  {
		//				if((ps4.button == LEFT))
		//				{
		//					while(ps4.button == LEFT);
		//					intake_f = !intake_f;
		//					KFS_R_cmd = intake_f ? 200 : 0 ;
		//
		//					RBMS_Set_Target_Velocity(KFS_R,KFS_R_cmd);
		//					KFS_L_cmd = intake_f ? 3000 : 0;
		//		//			comm_can2_set_rpm(KFS_L_ID,		intake_f ? 8000 : 0);
		//					//			RBMS_Set_Target_Velocity(KFS_L, intake_f ? -300 : 0);
		//		//			Ext_MoveTo(&kfs_grip,intake_f ? -0.76 : 0.0);
		//				}


		if(!PB1_IN)
		{
			while(!PB1_IN);
			intake_f = !intake_f;
			KFS_R_cmd = intake_f ? 200 : 0 ;

			RBMS_Set_Target_Velocity(KFS_R,KFS_R_cmd);
			KFS_L_cmd = intake_f ? 3000 : 0;
			//			comm_can2_set_rpm(KFS_L_ID,		intake_f ? 8000 : 0);
		}
		if(ps4.button == L1)
		{
			while(ps4.button == L1);
			if(s_pick_flag == 2)
			{
				Ext_SetMotionParams(&spear_picker, 0.4, 0.1, 0.1);
				Ext_MoveTo(&spear_picker,0.0);
				s_pick_flag = 0;
			}
			else if(s_pick_flag == 1)
			{
				Ext_SetMotionParams(&spear_picker, 0.7, 0.2, 0.2);
				Ext_MoveTo(&spear_picker,0.4);
				s_pick_flag += 1;
			}
			else{
				Ext_SetMotionParams(&spear_picker, 0.7, 0.2, 0.2);
				Ext_MoveTo(&spear_picker,0.75);
				s_pick_flag += 1;
			}
		}
		if(s_pick_flag == 0 && !spear_picker.is_moving)
		{
			rbms3.motor[RBMS4].config.POS_P = 1;
			rbms3.motor[RBMS4].config.cur_limit = 0.7;
		}
		else{
			rbms3.motor[RBMS4].config.POS_P = 300;
			rbms3.motor[RBMS4].config.cur_limit = 10;
		}
		if(ps4.button == R1)
		{
			while(ps4.button == R1);
			if(s_arm_flag)
			{
				spear_dir = 2;
				s_arm_flag = 0;
			}
			else
			{
				spear_dir = 1;
				s_arm_flag = 1;
			}
		}
		if(ps4.button == DOWN)
		{
			while(ps4.button == DOWN);
			Navigator_Stop(&path_plan);
			Navigator_Stop(&mc_pp);
			//			NavTraj_Stop(&mc_traj);
			//		pathplan.completed = 1;
			Ext.cmd = EXT_CMD_IDLE;
			Ext.climb_state = CLIMB_IDLE;
			Ext.descend_state = DESCEND_IDLE;
			KFS.pick_state = PICK_IDLE;
			KFS.place_state = PLACE_IDLE;
			PathApp.step_status = APP_IDLE;
			MC.cmd = MC_CMD_IDLE;
			MC.pick_state = MC_PICK_IDLE;
			navi_state = MANUAL;
		}
		//		if(fabs(ps4.joyR_y) > 0.05)
		//		{
		//			RBMS_Set_Target_Velocity(SpearArm, 50 * ps4.joyR_y);
		//		}
		//		else {
		//			RBMS_Set_Target_Velocity(SpearArm, 0);
		//		}
		//		if(ps4.button == DOWN)
		//		{
		//			while(ps4.button == DOWN);
		//			Can_Bridge_Tx_Yaw(IMU.real_z);
		//		}

		Hall[0] = DIG1_IN;
		Hall[1] = DIG2_IN;
		Hall[2] = DIG3_IN;
		Hall[3] = DIG4_IN;
		Hall[4] = readbit(F_FWRD);
		Hall[5] = readbit(UP_F);
		Hall[6] = readbit(DF_20);
		Hall[7] = readbit(UP_B);
		Hall[8] = readbit(DOWN_B);
		Hall[9] = readbit(SPEAR_IR);
		Hall[10] = readbit(KFS_IR);


		//		if(fabs(ps4.joyR_y) > 0.05)
		//		{
		//			RBMS_Set_Target_Velocity(BExt_RW, ps4.joyR_y *  250);
		//			RBMS_Set_Target_Velocity(BExt_LW, ps4.joyR_y * -250);
		//			RBMS_Set_Target_Velocity(FExt_RW, ps4.joyR_y *  250);
		//			RBMS_Set_Target_Velocity(FExt_LW, ps4.joyR_y * -250);
		//		}
		//		else if(Ext.cmd == EXT_CMD_IDLE){
		//			RBMS_Set_Target_Velocity(BExt_RW, 0);
		//			RBMS_Set_Target_Velocity(BExt_LW, 0);
		//			RBMS_Set_Target_Velocity(FExt_RW, 0);
		//			RBMS_Set_Target_Velocity(FExt_LW, 0);
		//		}
		//		encoderpos();
		if(ps4.button == SHARE)
		{
			while(ps4.button == SHARE);
			tangle = 90;
			angle_error = 0;
			PIDDelayInit(&pid_z);
		}
		static uint8_t led = 0;
		if (++led >= 255) {
			led3 = !led3;
			led = 0;
		}
  }
  /* USER CODE END 5 */
}

//int op_status[10] = {0};
//uint8_t addr_f = 0;
//int search_c = 0;
//HAL_StatusTypeDef probe_f;
//uint8_t tf_cmd = 0;

void SecondaryTask(void *argument) {
	Moving_Average_Init(&TF_F_MA,&Front_TF,&FilteredTF[0]);
	//	Moving_Average_Init(&TF_L_MA,&Left_TF,&FilteredTF[1]);
	Moving_Average_Init(&TF_B_MA,&Back_TF,&FilteredTF[2]);
	Moving_Average_Init(&TF_R_MA,&Right_TF,&FilteredTF[3]);
	uint8_t grip_f = 0;

	//		J60_motion_control(&j601, yawdeg/180*PI, 0.0, 0.0, 20.0, 6.0);
	while (1) {

		if(!PB2_IN)
		{
			while(!PB2_IN);
			//					op_status[5] = TF_SaveSettings(&TF_NOVA3);
			if(grip_f)
			{
				//						Ext_MoveTo(&kfs_grip, 0.0);
				Path_Profile_Set(&j60_prof,
						current_pos_j60,
						YAW_AD,
						KFS_YAW_0 - current_pos_j60,
						YAW_DD,
						YAW_V);


				grip_f = 0;


			}
			else{
				Ext_MoveTo(&kfs_grip, 0.45);
				Path_Profile_Set(&j60_prof,
						current_pos_j60,
						YAW_AD,
						KFS_YAW_180 - current_pos_j60,
						YAW_DD,
						YAW_V);

				grip_f = 1;
			}

		}
		if(!PB1_IN)
		{
			while(!PB1_IN);
			//			J60_SendCommand(&J60_1, J60_CMD_SAVE_CONFIG NULL, 0);
			osDelay(500);
			//			J60_ChangeID(&J60_1, 1);
			//			Path_Profile_Reset(&j60_prof);
			//
			//			   j60_prof.path = 0;
			//			   j60_prof.d = 0;
			//			   j60_prof.d1 = 0;
			//			   j60_prof.d2 = 0;
			//			   j60_prof.d3 = 0;
			//			   j60_prof.td = 0;
			//			   j60_prof.v = 0;
			//			   j60_prof.t = 0;
			//			   j60_prof.x = 0;

			//			   J60_SendCommand(&J60_1,J60_CMD_ENABLE,NULL,SEND_DLC_ENABLE_MOTOR);
			//			   osDelay(50);

		}
		//				TF_ReadData(&TF_NOVA, &NOVA_data);
		TF_Bus_Tick(&i2c5_bus);
		//		Left_TF  = TF_L_data.distance / 100.0f;
		Right_TF = TF_R_data.distance / 100.0f;
		Back_TF  = TF_B_data.distance / 100.0f;
		Front_TF = TF_F_data.distance / 100.0f;
		FilteredTF[1] = tfmini.f_dist;
		Moving_Average_Filter(&TF_F_MA);
		Moving_Average_Filter(&TF_B_MA);
		Moving_Average_Filter(&TF_R_MA);
		//		Moving_Average_Filter(&TF_L_MA);

		static uint8_t led = 0;
		if (++led >= 255) {
			led2 = !led2;
			led = 0;
		}
		osDelay(20);
	}
}

void fivems_Task(void * arg){ //5ms
	//	while(!localizer.laser_fresh);m
	//	Localizer_Init(&localizer);

	while(1)
	{
		osSemaphoreAcquire(fivemsSemaphore,osWaitForever);
		timer3  = tim13_cnt * 0.005f;

		/* --- Snapshot raw state under critical section --- */
		R2RawState_t snap;
		taskENTER_CRITICAL();
		snap = *(R2RawState_t *)&g_raw;   /* memcpy via assignment, volatile -> local */
		taskEXIT_CRITICAL();

		/* --- Cook into g_world --- */
		g_world.r2_x_m     = snap.r2_pose_x_mm * 0.001f;   /* mm -> m */
		g_world.r2_y_m     = snap.r2_pose_y_mm * 0.001f;
		g_world.r2_z_m     = snap.r2_pose_z_mm * 0.001f;
		g_world.r2_yaw_deg = snap.r2_yaw_cdeg  * 0.01f;    /* cdeg -> deg */

		g_world.r1.x    = snap.r1_pose_x_mm * 0.001f;
		g_world.r1.y    = snap.r1_pose_y_mm * 0.001f;
		g_world.r1.zone = (R1_Zone_t)snap.r1_zone;

		memcpy(g_world.forest, snap.forest, 12);
		g_world.current_block = snap.current_block;
		g_world.status_flags  = snap.status_flags;

		//		Localizer_Update_5ms(&localizer);

		//		if (path_result == R2CAN_COMPLETE) {
		////			App_OnPathReceived(&path);
		//		}
		//		else if (path_result == R2CAN_ERR_CRC) {
		//			/* Optional: signal error LED or increment error counter */
		//		}

		if(j60_prof.state != 0)
		{
			Path_Profile_Update(&j60_prof);
			current_pos_j60 = j60_prof.path;
			//			J60_rotation_motion_ctrl(&j602, current_pos_j60, 0.0, 0.0, 200.0, 6.0);
			J60_SetMotion(&J60_1, current_pos_j60, 0.0, 200.0, 6.0, 0.0);

		}
		if(front_ext.is_moving)
		{
			front_cmd = Ext_Update(&front_ext);
			RBMS_Set_Target_Position(FExt_M, front_cmd);
		}
		if(kfs_yaw.is_moving)
		{
			kfs_yaw_cmd = Ext_Update(&kfs_yaw);
			RBMS_Set_Target_Position(KFS_YAW, kfs_yaw_cmd);
		}
		if(rear_ext.is_moving)
		{
			rear_cmd = Ext_Update(&rear_ext);
			RBMS_Set_Target_Position(BExt_M, (rear_cmd));
		}
		//		if(spear_arm.is_moving)
		//		{
		//			sarm_cmd  = Ext_Update (&spear_arm);
		//			RBMS_Set_Target_Position(SpearArm, sarm_cmd);
		//		}

		if(spear_picker.is_moving)
		{
			sgripper_cmd  = Ext_Update (&spear_picker);
			RBMS_Set_Target_Position(SpearGripper, sgripper_cmd);
		}
		if(kfs_up.is_moving)
		{
			kfs_z_cmd  = Ext_Update (&kfs_up);
			RBMS_Set_Target_Position(KFS_Z, kfs_z_cmd);
		}
		if(kfs_grip.is_moving)
		{
			kfs_g_cmd  = Ext_Update (&kfs_grip);
			RBMS_Set_Target_Position(KFS_G, kfs_g_cmd);
		}
		comm_can2_set_rpm(KFS_L_ID,KFS_L_cmd);
		spearHandler();

		ApplySteps(&PathApp);
	}
}
void spearHandler()
{
	if(spear_dir == 1)
	{
		RBMS_Set_Control_Mode(SpearArm, VELOCITY);
		RBMS_Set_Target_Velocity(SpearArm, 150);
		spear_dir = -1;
	}
	else if(spear_dir == 2)
	{
		RBMS_Set_Control_Mode(SpearArm, VELOCITY);
		RBMS_Set_Target_Velocity(SpearArm, -110);
		spear_dir = -1;

	}
	else if(spear_dir == -1)
	{
		if(fabs(rbms3.motor[2].current) > 9.0)
		{
			RBMS_Set_Control_Mode(SpearArm, VELOCITY);
			RBMS_Set_Target_Velocity(SpearArm, 0);
			spear_dir = 0;
		}
	}
	else{
		rbms3.motor[RBMS3].config.cur_limit = 10.0;
		rbms3.motor[RBMS3].config.POS_P = 300;
		RBMS_Set_Control_Mode(SpearArm, POSITION);
		RBMS_Set_Target_Position(SpearArm, 0);
	}
}
//void Path_Handler(const R2CAN_Path *path)
//{
//	uint8_t i;
//	R2CAN_Step *s;
//	for (i = 0; i < path->step_count ; ) {
//		s = (R2CAN_Step *)&path->steps[i];
//
//		switch (s->action_type) {
//
//		case R2CAN_ACT_PRE_ENTRY:
//			/* Pick up R2 box from s->target_block before entering grid */
//			/* Motor_PickupExternal(s->target_block); */
//			break;
//
//		case R2CAN_ACT_MOVE:
//			if (s->requires_r1_clear) {
//				/* Signal R1 teammate, wait for acknowledgement */
//				/* CAN_SendR1ClearRequest(s->target_block); */
//			}
//			/* Rotate to s->direction, move forward one block */
//			/* Motor_RotateTo(s->direction); */
//			/* Motor_MoveForward();          */
//			if (s->auto_pickup) {
//				/* Stepped onto R2 box — trigger pickup mechanism */
//				/* Pickup_Trigger(); */
//			}
//			break;
//
//		case R2CAN_ACT_PICKUP:
//			/* Stay in place, rotate to face s->direction, pick up */
//			/* Motor_RotateTo(s->direction); */
//			/* Pickup_Trigger();             */
//			break;
//
//		default:
//			break;
//		}
//
//		/* s->collected_after tells you the running R2 box count (0-3) */
//	}
//}
double ts_pd = 0.0;

/* USER CODE BEGIN Header_CalculationTaskEntry */
/**
* @brief 	Function implementing the CalculationTask thread.
* @note		Triggers every 5ms
*/

void CalculationTaskEntry(void *argument)
{

  while(1)
  {

		osSemaphoreAcquire(calcSemaphore, osWaitForever);
		if(ps4.button == OPTION){
			while(ps4.button == OPTION);
			NVIC_SystemReset();

		}

		RBMS_5ms(&rbms1);
		RBMS_5ms(&rbms2);
		RBMS_5ms(&rbms3);
		RBMS_5ms(&rbms4);

		if(swerveA.aligned && swerveB.aligned && swerveC.aligned && swerveD.aligned){
			myswerve.aligned = 1;
			myswerve.align_flag = 0;
		}
		else{
			myswerve.aligned = 0;
			myswerve.align_flag = 1;
		}
		//		op_status[3] = TF_ReadData(&TF_F,&TF_F_data);
		//		op_status[2] = TF_ReadData(&TF_R,&TF_R_data);

		//		op_status[4] = TF_ReadData(&TF_NOVA3,&NOVA_data);
		//				TF_ReadData(&TF_NOVA,&NOVA_data);
		//		TF_Bus_Tick(&i2c2_bus);

		encoderpos();
		//		Navigator_Update(&pathplan);
		Navigator_Update(&mc_pp);
		//		double dt = timer - traj_prev_timer;
		//		ts_pd = dt;
		//		traj_prev_timer = timer;
		//
		//		if (dt > 0.0 && dt < 0.1) {
		//		    NavFIR_Update(&mc_fir, (float)dt);
		//		}

		navi();


	  static uint8_t led = 0;
	  if (++led > 4) {
		led3 = !led3;
		led = 0;
	  }

  }
}

void SerialTask (void *argument)
{

	static char buffer[100];
	HAL_UART_Receive_IT(&huart5, &rxChar, 1);
	while(1)
	{
		if(msg_received == 1)
		{
			Parse3Floats(rxBuffer);
			snprintf(buffer,sizeof(buffer),"Set successfully! \r\n");
			UART_SendString_IT_blocking_safe(buffer);
			msg_received = 0;
			osDelay(1000);
		}

		else{
			//			snprintf(buffer,sizeof(buffer),"X:%d \r\n",NOVA_data.distance);
			//			snprintf(buffer,sizeof(buffer),"F:%.3f, L:%.3f,path: %.3f,s: %d, T:%.3f \r\n",fFront_TF,fLeft_TF,mc_pp.prof_y.path,mc_pp.prof_y.state, MC.pick_time);
			snprintf(buffer,sizeof(buffer),"F:%.3f, L:%.3f,time:%.3f\r\n",FilteredTF[0],FilteredTF[1],MC.pick_time);

			//						snprintf(buffer,sizeof(buffer),"X:%.3f , Y:%.3f\r\n",actual_x,actual_y);
			//									snprintf(buffer,sizeof(buffer),"Grip:%.3f, Curr:%.3f, Moving:%d\r\n",rbms4.motor[2].pos,rbms4.motor[2].current,kfs_grip.is_moving);
			//			snprintf(buffer,sizeof(buffer),"Yaw:%.3f, Z:%.3f, G:%.3f\r\n",rbms4.motor[1].pos,rbms4.motor[3].pos,rbms4.motor[2].pos);
			UART_SendString_IT_blocking_safe(buffer);
			osDelay(100);
		}
		osDelay(10);
	}
}
void align1(void *argument){
	MonoSwerve_Init(&swerveA,&rbms1,RBMS1,&GPIOA->IDR,10,NULL,2.0f,33.57);
	osDelay(500);
	if(MonoSwerve_Align(&swerveA))
	{
		swerveA.aligned = 1;
	}
	while(1){
		if(myswerve.align_flag)
		{
			if(MonoSwerve_Align(&swerveA))
			{
				swerveA.aligned = 1;
			}
		}
		osDelay(10);
	}
}

void align2(void *argument){

	MonoSwerve_Init(&swerveB,&rbms1,RBMS2,&GPIOB->IDR,2,NULL,2.0f,119.33);
	osDelay(500);
	if(MonoSwerve_Align(&swerveB))
	{
		swerveB.aligned = 1;
	}
	while(1){
		if(myswerve.align_flag)
		{
			if(MonoSwerve_Align(&swerveB))
			{
				swerveB.aligned = 1;
			}
		}
		osDelay(10);

	}
}
void align3(void *argument){
	MonoSwerve_Init(&swerveC,&rbms1,RBMS3,&GPIOC->IDR,0,NULL,2.0f,26.84);
	osDelay(500);
	if(MonoSwerve_Align(&swerveC)){
		swerveC.aligned = 1;
	}
	while(1){
		if(myswerve.align_flag)
		{
			if(MonoSwerve_Align(&swerveC))
			{
				swerveC.aligned = 1;
			}
		}
		osDelay(10);
	}
}
void align4(void *argument){
	MonoSwerve_Init(&swerveD,&rbms1,RBMS4,&GPIOB->IDR,3,NULL,2.0f,196.37);
	osDelay(500);
	if(MonoSwerve_Align(&swerveD))
	{
		swerveD.aligned = 1;
	}
	while(1){
		if(myswerve.align_flag)
		{
			if(MonoSwerve_Align(&swerveD))
			{
				swerveD.aligned = 1;
			}
		}
		osDelay(10);
	}
}

void MC_Task (void *arg)
{
	//	Navigator_Init(&mc_pp,&localizer.kf_y.x,&localizer.kf_x.x, &IMU.real_zrad);
	Navigator_Init(&mc_pp, (float *)&FilteredTF[1],(float*) &FilteredTF[0], (float *)&IMU.real_zrad);
	Nav_SetMotionParams(&mc_pp,5.0,0.2,0.3);
	Navigator_TunePID_Position(&mc_pp, 3.0, 0.01, 1.0);
	Nav_SetPosTol(&mc_pp,0.0003);


	Ext_Init(&spear_picker, SPEAR_PICKER_POS);
	Ext_Init(&spear_arm,    SPEAR_ARM_POS);
	Ext_SetMotionParams(&spear_arm, 3.0, 1.5, 1.5);
	Ext_SetMotionParams(&spear_picker, 1.5, 0.2, 0.2);
	Ext_SetTolerance(&spear_picker, 0.02);
	Ext_SetTolerance(&spear_arm, 0.025);
	TFmini_Init(&tfmini, &huart4);
	//	Navigator_Init(&pathplan,(float*) &actual_y,(float*) &actual_x , &IMU.real_zrad);
	//	NavFIR_Init(&mc_fir, (float *)&fLeft_TF,(float*) &fFront_TF, (float *)&IMU.real_zrad,0.005);
	//	NavFIR_SetMotionParams(&mc_fir, 1.5f, 2.0f, 1.0f, 0.1f);
	//	NavFIR_SetPosTolerance(&mc_fir,0.002);
	//	mc_kp_x = 1.0;
	//	mc_ki_x = 0.002;
	//	mc_kd_x = 0.1;
	//	mc_kn_x = 50.0;


	mc_kp_x = 1.0;
	mc_ki_x = 0.005;
	mc_kd_x = 0.13;
	mc_kn_x = 13.0;

	//	mc_kp_y = 1.5;
	//	mc_ki_y = 0.005;
	//	mc_kd_y = 0.079;
	//	mc_kn_y = 80.0;

	mc_vel = 0.7;
	mc_accel_d = 0.2;
	mc_decel_d = 0.2;

	while(1){

		if(ps4.button == LEFT)
		{
			while(ps4.button == LEFT);
			if(MC.cmd == MC_CMD_IDLE){
				MC.cmd = MC_CMD_PICK ;
			}
		}

		switch(MC.cmd)
		{
		case MC_CMD_IDLE:
			break;
		case MC_CMD_GOTO_ENTRY:
			break;
		case MC_CMD_BUSY:
			break;
		case MC_CMD_PICK:
			if(MC.pick_state == MC_PICK_IDLE)
			{
				MC.pick_state = MC_PICK_Start;
				MC.cmd = MC_CMD_BUSY;
			}
			break;
		default:
			break;
		}

		switch(MC.pick_state)
		{
		case MC_PICK_IDLE:
			break;
		case MC_PICK_Start: 	/* Robot pathplanes to the first point */
			//			navi_state = NAVI_MF;
			//			tVy = -0.2;
			//			MC.timer = timer;
			//			MC.pick_state = MC_PICK_CLEAR_SENSOR;
			navi_state = NAVI_MC;
			tangle = 90;
			Nav_SetMotionParams(&mc_pp,mc_vel,mc_accel_d,mc_decel_d);
			//		    Navigator_TunePID_Position(&mc_pp, mc_kp, mc_ki, mc_kd);
			PIDGainSet(KP, mc_kp_x, &mc_pp.pid_x);
			PIDGainSet(KD, mc_kd_x, &mc_pp.pid_x);
			PIDGainSet(KN, mc_kn_x, &mc_pp.pid_x);


			PIDGainSet(KP, mc_kp_x, &mc_pp.pid_y);
			PIDGainSet(KD, mc_kd_x, &mc_pp.pid_y);
			PIDGainSet(KN, mc_kn_x, &mc_pp.pid_y);

			Navigator_LockPositionAt(&mc_pp, spear_point[MC.target_point].x, spear_point[MC.target_point].y, NAN);

			Ext_SetMotionParams(&spear_picker, 1.5, 0.2, 0.2);
			Ext_MoveTo(&spear_picker,0.6);
			/* Implementation code for spear arm movement*/
			spear_dir = 2; //move it backward
			MC.pick_state = MC_PICK_ARM_OUT;
			//			MC.pick_state = MC_PICK_ERR;
			MC.pick_time = timer;
			break;
		case MC_PICK_CLEAR_SENSOR:
			if(timer - MC.timer > 0.5){
				tVy = 0.0;
				//				navi_state = NAVI_MC;
				//				Navigator_MoveTo(&mc_pp, spear_point[MC.target_point].x, spear_point[MC.target_point].y, NAN);
				//				Ext_MoveTo(&spear_picker,0.0);
				//				/* Implementation code for spear arm movement*/
				//				spear_dir = 2; //move it backward
				//				MC.pick_state = MC_PICK_ARM_OUT;
			}
			break;
		case MC_PICK_ARM_OUT: 	/* Arm is out 270° */
			if(!mc_pp.is_moving && !spear_picker.is_moving && spear_dir == 0)
			{
				Ext_SetMotionParams(&spear_picker, 1.0, 0.2, 0.2);
				Ext_MoveTo(&spear_picker, PICKER_270);
				s_pick_flag = 2;
				MC.pick_time = fabs(MC.pick_time - timer);
				MC.pick_state = MC_PICK_ERR;
//				MC.pick_state = MC_PICK_CONFIRMING;

			}
			break;
		case MC_PICK_CONFIRMING:
			if(!spear_picker.is_moving && !mc_pp.is_moving)
			{
				if(readbit(SPEAR_IR))
				{
					navi_state = NAVI_MF;
					tVy = 0;
					tVx = 0;
					MC.target_point ++;
					MC.pick_state = MC_PICK_GRAB;
				}

				else
				{
					MC.pick_state = MC_PICK_ERR;
					if(MC.target_point < 4){
						MC.target_point ++;
						MC.pick_state = MC_PICK_MOVE_NEXT;
					}
				}
			}
			break;
		case MC_PICK_MOVE_NEXT:
			if(MC.target_point < 5){
				//				Nav_SetMotionParams(&mc_pp,2.0,0.2,0.3);
				//				Navigator_TunePID_Position(&mc_pp, 3.0, 0.005, 0.5);
				navi_state = NAVI_MC;
				Navigator_LockPositionAt(&mc_pp, spear_point[MC.target_point].x, spear_point[MC.target_point].y, NAN);
				MC.pick_state = MC_PICK_CONFIRMING;
			}
			break;
		case MC_PICK_GRAB: 		/* Robot moves into the spear rack to grab the spearhead*/
			if(!spear_picker.is_moving)
			{
				navi_state = NAVI_MF;
				tVx = 0.25; // THIS IS IN ROBOT FRAME (THE ROBOT IS ROTATED TO FACE THE SPEARHEAD)
				spear_dir = 2; //move it backward
				MC.timer = timer;
				MC.pick_state = MC_PICK_PICKUP;
			}
			break;
		case MC_PICK_PICKUP: 	/* Robot rotates spearhead */
			if((timer - MC.timer) > 0.75)
			{
				MC.pick_time = MC.pick_time - timer;
				tVx = -0.2; // THIS IS IN ROBOT FRAME (THE ROBOT IS ROTATED TO FACE THE SPEARHEAD)
				spear_dir = 2; //move it backward
				osDelay(50);
				tVx = 0.0; // THIS IS IN ROBOT FRAME (THE ROBOT IS ROTATED TO FACE THE SPEARHEAD)
				Ext_MoveTo(&spear_picker, 0.4);
				s_pick_flag = 1;
				MC.pick_state = MC_PICK_TRANSPORT;

			}
			break;
		case MC_PICK_TRANSPORT:
			if(!spear_picker.is_moving){
				spear_dir = 1; //Move spear arm back
				MC.pick_state = MC_PICK_ASSEMBLE_POS;
			}
			break;
		case MC_PICK_ASSEMBLE_POS:
			if(spear_dir == 0){
				Ext_SetMotionParams(&spear_picker, 0.8, 0.2, 0.2);
				Ext_MoveTo(&spear_picker, 0.0);
				s_pick_flag = 0;
				MC.pick_state = MC_PICK_End;
			}
			break;
		case MC_PICK_MOVE:		/* Robot Goes to assembly point, moving the spearhead to the other side*/
			if(!spear_picker.is_moving)
			{
				navi_state = NAVI_MC;
				Navigator_MoveTo(&mc_pp, 0.4, 0.4, NAN);
				/*Implementation code for moving spear arm*/
				spear_dir = 2; //Move spear arm back
			}
			break;
		case MC_PICK_ASSEMBLE:	/* Lock Position, if the speararm_ext stopped moving then immediately start the rotation of the spearpicker to 0° position*/
			if(!mc_pp.is_moving)
			{
				Navigator_LockPosition(&mc_pp);
				MC.pick_state = MC_PICK_End;
			}
			break;
		case MC_PICK_End:
			if(MC.assembled)
			{
				MC.pick_state = MC_PICK_IDLE;
				MC.cmd = MC_CMD_IDLE;
				navi_state = MANUAL;
			}
			break;
		case MC_PICK_ERR:
			break;
		}
	}


}


void KFS_Task(void *argument)
{

	//	Ext_Init(&kfs_yaw,&rbms4.motor[3].pos);
	Ext_Init(&kfs_up,KFS_Z_POS);
	Ext_Init(&kfs_grip,KFS_G_POS);
	//	Ext_Init(&cubemars_ext,    &cubemars1.pos);
	Ext_Init(&kfs_yaw, KFS_YAW_POS);
	Ext_SetMotionParams(&kfs_grip,1.4,0.1,0.1);
	Ext_SetMotionParams(&kfs_up, 2.5, 0.7, 0.4);
	Ext_SetMotionParams(&kfs_yaw, 0.6, 0.2, 0.4);
	Ext_SetTolerance(&kfs_grip, 0.1);
	Ext_SetTolerance(&kfs_up, 0.06);
	Ext_SetTolerance(&kfs_yaw, 0.1);
	//	Ext_SetTolerance(&cubemars_ext, 1.0);
	//	Ext_SetMotionParams(&cubemars_ext, 100.0, 30.0, 40.0);
	//	uint8_t kfs_z_state = 0;
	Path_Profile_Init(&j60_prof);
	J60_Init(&J60_1,&h7fdcan2, J60_ID);
	osDelay(500);
	J60_Enable(&J60_1);
	osDelay(500);
	uint8_t j60_f = 0;
	uint8_t kfs_up_f = 0;
	while(1)
	{
		/*--------------------------------------------------------------------------------------------------------------------------------------*/
		/*                                                         PICKING ALGORITHM                                                            */
		/*--------------------------------------------------------------------------------------------------------------------------------------*/
		//		if(ps4.button == UP)
		//		{
		//			while(ps4.button == UP);
		//			kfs_z_state = !kfs_z_state;
		//			Ext_MoveTo(&kfs_up,(kfs_z_state ? KFS_Z_MAX : 0.1));
		//		}
		if(ps4.button == RIGHT)
		{
			while(ps4.button == RIGHT);
			KFS.cmd = KFS_CMD_PICK;
			KFS.level = KFS_LEVEL_20;
			KFS.direction = KFS_DIR_FORWARD;
			KFS.auto_pickup = 0;
		}
		if(ps4.button == L3)
		{
			while(ps4.button == L3);
			KFS.cmd = KFS_CMD_PLACE;
			KFS.level = KFS_LEVEL_TOP;
			KFS.direction = KFS_DIR_FORWARD;
			KFS.auto_pickup = 0;
		}
//		if(ps4.button == RIGHT)
//		{
//			while(ps4.button == RIGHT);
//			if(j60_f){
//				Path_Profile_Set(&j60_prof,
//						current_pos_j60,
//						YAW_AD,
//						KFS_YAW_0 - current_pos_j60,
//						YAW_DD,
//						YAW_V);
//				j60_f = 0;
//			}
//			else{
//				Path_Profile_Set(&j60_prof,
//						current_pos_j60,
//						YAW_AD,
//						KFS_YAW_180 - current_pos_j60,
//						YAW_DD,
//						YAW_V);
//				j60_f = 1;
//			}
//		}
//		if(ps4.button == L3)
//		{
//			while(ps4.button == L3);
//			kfs_up_f = !kfs_up_f;
//			Ext_MoveTo(&kfs_up, kfs_up_f ? KFS_Z_MAX : KFS_Z_0);
//		}
		//		if(ps4.button == LEFT)
		//		{
		//			while(ps4.button == LEFT);
		//			KFS.cmd = KFS_CMD_PICK;
		//			KFS.level = KFS_LEVEL_BELOW;
		//			KFS.direction = KFS_DIR_FORWARD;
		//			KFS.auto_pickup = 0;
		//		}


		//		if(ps4.button == L3)
		//		{
		//			while(ps4.button == L3);
		//			KFS.cmd = KFS_CMD_PLACE;
		//			KFS.level = KFS_LEVEL_MID;
		//			KFS.direction = KFS_DIR_FORWARD;
		//		}
		//		if(ps4.button == L3)
		//		{
		//			while(ps4.button == L3);
		//			KFS.cmd = KFS_CMD_PICK;
		//			KFS.level = KFS_LEVEL_20;
		//			KFS.direction = KFS_DIR_FORWARD;
		//			KFS.auto_pickup = 1;
		//		}
		switch(KFS.cmd)
		{
		case KFS_CMD_PICK:
			if(KFS.pick_state == PICK_IDLE)
			{
				KFS.auto_pickup = 0;
				KFS.pick_state = PICK_Start;
				KFS.cmd = KFS_CMD_BUSY;
			}
			break;
		case KFS_CMD_PLACE:
			if(KFS.place_state == PLACE_IDLE)
			{
				KFS.auto_pickup = 0;
				KFS.place_state = PLACE_Start;
				KFS.cmd = KFS_CMD_BUSY;
			}
			break;
		case KFS_CMD_AUTOPICKUP:
			if(KFS.place_state == PLACE_IDLE)
			{
				KFS.auto_pickup = 1;
				KFS.place_state = PLACE_Start;
				KFS.cmd = KFS_CMD_BUSY;
			}
			break;
		case KFS_CMD_IDLE:
			break;
		default:
			break;
		}

		switch(KFS.pick_state)
		{
		case PICK_IDLE: /*The arm should be at max height and at 0° with a closed grip*/
			break;
		case PICK_Start:
			//			switch(KFS.direction)
			//			{
			//			case KFS_DIR_LEFT:
			//				tangle = 90.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case KFS_DIR_FORWARD:
			//				tangle = 0.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case KFS_DIR_RIGHT:
			//				tangle = -90.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case KFS_DIR_BACK:
			//				tangle = -180.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			default:
			//				break;
			//
			//			}
			Ext_MoveTo(&kfs_up,KFS_Z_MAX); // Height
			Ext_MoveTo(&kfs_grip,KFS_G_MAX); //Grip
			/*---IMPLEMENTATION CODE FOR ROTATION---*/
			//			Path_Profile_Set(&j60_prof,
			//					current_pos_j60,
			//					YAW_AD,
			//					KFS_YAW_0 - current_pos_j60,
			//					YAW_DD,
			//					YAW_V);
			KFS.pick_state = PICK_ARM_OUT;
			break;
		case PICK_ARM_OUT: /*The arm should rotate to 180°*/
			if(!kfs_up.is_moving && !kfs_grip.is_moving && (j60_prof.state == 0))
			{
				Path_Profile_Set(&j60_prof,
						current_pos_j60,
						YAW_AD,
						KFS_YAW_180 - current_pos_j60,
						YAW_DD,
						YAW_V);
				osDelay(10);
				KFS.pick_state = PICK_START_INTAKE;
			}
			break;
		case PICK_START_INTAKE: /*The arm should open and start the intake motors*/
			if(j60_prof.state == 0)
			{
				Ext_MoveTo(&kfs_grip, KFS_G_MAX);
				RBMS_Set_Target_Velocity(KFS_R, 300);
				KFS_L_cmd = 3500;
				//					RBMS_Set_Target_Velocity(KFS_L,-300);
				//					comm_can2_set_rpm(KFS_L_ID, 8000);
				KFS.pick_state = PICK_Z_LEVEL;
			}
			break;
		case PICK_Z_LEVEL: /*The arm should move to the Z level of the target block - get this from height_delta_enc*/
			if(!kfs_grip.is_moving)
			{
				if(KFS.level == KFS_LEVEL_20)
				{
					if(KFS.stored == 2){
						Ext_MoveTo(&kfs_up,KFS_Z_S2);
					}
					else{
						Ext_MoveTo(&kfs_up,KFS_Z_20);
					}
				}
				else if(KFS.level == KFS_LEVEL_40)
				{
					Ext_MoveTo(&kfs_up,KFS_Z_40);
				}
				else if(KFS.level == KFS_LEVEL_BELOW)
				{
					Ext_MoveTo(&kfs_up,KFS_Z_0);
				}
				else
				{
					KFS.pick_state = PICK_ERR;
					break;
				}
				KFS.pick_state = PICK_GRIP;
			}
			break;
		case PICK_GRIP: /*The arm should grip the KFS*/
			if(!kfs_up.is_moving)
			{
				Ext_MoveTo(&kfs_grip, KFS_G_V);
				KFS.timer = timer;
				KFS.pick_state = PICK_STOP_INTAKE;
			}
			break;
		case PICK_STOP_INTAKE:/*The arm should stop the intake motors when the IR sensor detects the KFS
								(we can try to tighten the grip here a bit more)*/
			if((!readbit(KFS_IR) || (!kfs_grip.is_moving)) && (timer - KFS.timer > 2.0))
			{
				//				Ext_MoveTo(&kfs_grip, KFS_G_Grip);
				RBMS_Set_Target_Position(KFS_G,KFS_G_Tight);
				RBMS_Set_Target_Velocity(KFS_R, 0);
				KFS_L_cmd = 0;
				//					RBMS_Set_Target_Velocity(KFS_L, 0);
				//					comm_can2_set_rpm(KFS_L_ID, 0);
				KFS.timer = timer;
				KFS.pick_state = PICK_UP_MAX;
			}

			break;
		case PICK_UP_MAX: /*The arm should go to the max height*/
			if(timer - KFS.timer > 0.5)
			{
				if(KFS.auto_pickup && Ext.cmd == EXT_CMD_IDLE)
				{
					Ext.cmd = EXT_CMD_FORWARD;
				}
				Ext_MoveTo(&kfs_up, KFS_Z_MAX);
				KFS.pick_state = PICK_ARM_IN;
			}
			break;
		case PICK_ARM_IN: /*The arm should rotate to 0°*/
			if(!kfs_up.is_moving)
			{
				/*---IMPLEMENTATION CODE FOR ROTATION---*/
				KFS.stored += 1;
				if(KFS.stored < 3){
					Path_Profile_Set(&j60_prof,
							current_pos_j60,
							YAW_AD,
							KFS_YAW_0 - current_pos_j60,
							YAW_DD,
							YAW_V);
				}
				else{
					Path_Profile_Set(&j60_prof,
							current_pos_j60,
							YAW_AD,
							0.628 - current_pos_j60,
							YAW_DD,
							YAW_V);
				}
				KFS.pick_state = PICK_STORE_LEVEL;
			}
			break;
		case PICK_STORE_LEVEL: /*The arm should move to the Z level for storage which is dependent on the number of KFS Stored*/
			if(j60_prof.state == 0)
			{
				KFS.pick_state = PICK_STORE_OUT;
				if(KFS.stored == 1)
				{
					Ext_MoveTo(&kfs_up,KFS_Z_STORE_1);
				}
				else if(KFS.stored == 2)
				{
					Ext_MoveTo(&kfs_up,KFS_Z_STORE_2);
				}
				else if(KFS.stored == 3)
				{
					Ext_MoveTo(&kfs_up,KFS_Z_MAX);
					KFS.pick_state = PICK_End;
				}
			}
			break;
		case PICK_STORE_OUT:
			if(!kfs_up.is_moving){
				KFS.pick_state = PICK_RELEASE;
				//			Ext_MoveTo(&kfs_grip,KFS_G_V);
				RBMS_Set_Target_Position(KFS_G, 0.4);
				RBMS_Set_Target_Velocity(KFS_R, -300);
				KFS_L_cmd = -3500;
				KFS.timer = timer;
			}
			break;
		case PICK_RELEASE: /*The arm should release, if this is the third KFS then it won't be released*/
			if(timer - KFS.timer > 1.0)
			{
				RBMS_Set_Target_Velocity(KFS_R, 0);
				KFS_L_cmd = 0;
				if(KFS.stored < 3){
					Ext_MoveTo(&kfs_grip,KFS_G_MAX);
					KFS.timer = timer;
				}
				KFS.pick_state = PICK_RETURN;
			}
			break;
		case PICK_RETURN:
			if(!kfs_grip.is_moving && (timer - KFS.timer > 1.0))
			{
				Ext_MoveTo(&kfs_up,KFS_Z_MAX);

				KFS.pick_state = PICK_End;
			}
			break;
		case PICK_End: /*Move back to neutral position (Max Height, Closed Grip, Intake Off), the scenario is different for the third KFS*/
			if(!kfs_grip.is_moving && !kfs_up.is_moving)
			{

				if(KFS.stored < 3)
				{
					KFS.pick_state = IDLE;
					//						Ext_MoveTo(&kfs_grip,0.0);
				}
				if(KFS.increment)
				{
					PathApp.step_status = APP_DONE;
				}
				KFS.cmd = KFS_CMD_IDLE;
			}
			break;
		default:
			break;
		}

		/****************************************************************************************************************************************/
		/*                                                         PLACING ALGORITHM                                                            */
		/****************************************************************************************************************************************/

		switch(KFS.place_state)
		{
		case PLACE_IDLE:
			break;
		case PLACE_Start: /*The arm should be at max height and at 0° with a closed grip*/
			//			switch(KFS.direction)
			//			{
			//			case KFS_DIR_LEFT:
			//				tangle = 90.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case KFS_DIR_FORWARD:
			//				tangle = 0.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case KFS_DIR_RIGHT:
			//				tangle = -90.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case KFS_DIR_BACK:
			//				tangle = -180.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			default:
			//				break;

			//			}
			KFS.place_state = PLACE_OPEN;
			if(KFS.stored == 3)
			{
				KFS.place_state = PLACE_ARM_OUT_90;
			}
			break;
		case PLACE_OPEN: /*The arm should open up to the maximum (inside the robot)*/
			Ext_MoveTo(&kfs_grip,KFS_G_MAX);
			KFS.place_state = PLACE_STORE_LEVEL;
			break;
		case PLACE_STORE_LEVEL:  /*The arm should go to the Z level where the last KFS was stored*/
			if(!kfs_grip.is_moving)
			{
				if(KFS.stored <= 1)
				{
					Ext_MoveTo(&kfs_up, KFS_Z_STORE_1);
				}
				if(KFS.stored == 2)
				{
					Ext_MoveTo(&kfs_up, (KFS_Z_STORE_2));
				}
				KFS.place_state = PLACE_GRIP;
			}
			break;
		case PLACE_GRIP: /*The arm should grip the KFS*/
			if(!kfs_up.is_moving)
			{
				Ext_MoveTo(&kfs_grip, KFS_G_V );
				KFS.place_state = PLACE_UP_MAX;
			}
			break;
		case PLACE_UP_MAX: /*The arm should go to the max height*/
			if(!kfs_grip.is_moving)
			{
				Ext_MoveTo(&kfs_up, KFS_Z_MAX);
				KFS.place_state = PLACE_ARM_OUT_90;
			}
			break;
		case PLACE_ARM_OUT_90: /*The arm should rotate to 90°*/
			if(!kfs_up.is_moving)
			{
				/*---IMPLEMENTATION FOR ROTATION---*/
				Path_Profile_Set(&j60_prof,
						current_pos_j60,
						YAW_AD,
						KFS_YAW_100 - current_pos_j60,
						YAW_DD,
						YAW_V);
				KFS.stored--;
//				KFS.place_state = PLACE_KFS_LEVEL;
				KFS.place_state = PLACE_LOOSEN_GRIP;

			}
			break;
		case PLACE_KFS_LEVEL: /*The arm should move to the required Z position for placement*/
			if(j60_prof.state == 0)
			{
				KFS.place_state = PLACE_ROBOT_EXT;
				if(KFS.level == KFS_LEVEL_MID)
				{
					//						Ext_MoveTo(&kfs_up,KFS_RACK_M);
					RBMS_Set_Target_Position(KFS_Z, KFS_RACK_M);
				}
				else if(KFS.level == KFS_LEVEL_TOP)
				{
					//						Ext_MoveTo(&kfs_up,KFS_RACK_T);
					RBMS_Set_Target_Position(KFS_Z, KFS_RACK_M);
				}
				else{
					KFS.place_state = PLACE_ERR;
				}
			}
			break;
		case PLACE_ROBOT_EXT:
			if(Ext.cmd == EXT_CMD_IDLE)
			{
				Ext.cmd = EXT_CMD_BUSY;
				if(KFS.level == KFS_LEVEL_MID){
					Ext_MoveTo(&front_ext, EXT_RACK_M);
					Ext_MoveTo(&rear_ext, -EXT_RACK_M);
				}
				else if(KFS.level == KFS_LEVEL_TOP)
				{
					Ext_MoveTo(&front_ext, EXT_RACK_T);
					Ext_MoveTo(&rear_ext, -EXT_RACK_T);
				}
				KFS.place_state = PLACE_ARM_OUT;
			}
			break;
		case PLACE_ARM_OUT: /*The arm should rotate to 180°*/
			if(!kfs_up.is_moving && !front_ext.is_moving)
			{
				/*---IMPLEMENTATION FOR ROTATION---*/
				Path_Profile_Set(&j60_prof,
						current_pos_j60,
						YAW_AD,
						KFS_YAW_180 - current_pos_j60,
						YAW_DD,
						YAW_V);
				KFS.place_state = PLACE_LOOSEN_GRIP;
			}
			break;
		case PLACE_LOOSEN_GRIP:
			if(j60_prof.state == 0)
			{
				RBMS_Set_Target_Position(KFS_G, 0.42);
				KFS.place_state = PLACE_KFS_OUT;
			}
			break;
		case PLACE_KFS_OUT:  /*The arm should start the intake motors in reverse to place the KFS (the grip can be loosened here a bit if needed)*/
			if(!kfs_grip.is_moving)
			{
				RBMS_Set_Target_Velocity(KFS_R,-300);
				//					RBMS_Set_Target_Velocity(KFS_L, 300);
				KFS_L_cmd = -3300;
				KFS.timer = timer;
				KFS.place_state = PLACE_ARM_IN_90;
			}
			break;
		case PLACE_ARM_IN_90: /* The arm should rotate to 90°, Stop the Intake motors*/
			if((timer - KFS.timer > 2.5) /*&& IR Sensor stops detecting*/ )
			{
				RBMS_Set_Target_Velocity(KFS_R, 0);
				//					RBMS_Set_Target_Velocity(KFS_L, 0);
				KFS_L_cmd = 0;
				/*---IMPLEMENTATION FOR ROTATION---*/
				Path_Profile_Set(&j60_prof,
						current_pos_j60,
						YAW_AD,
						KFS_YAW_0 - current_pos_j60,
						YAW_DD,
						YAW_V);
				KFS.place_state = PLACE_BACK_UP;
			}
			break;
		case PLACE_BACK_UP: /*The arm should go to max height*/
			if(j60_prof.state == 0)
			{
				Ext_MoveTo(&front_ext,Ext_conv(-1.0));
				Ext_MoveTo(&rear_ext,-Ext_conv(-1.0));
				Ext_MoveTo(&kfs_up,KFS_Z_MAX);
				KFS.place_state = PLACE_ARM_IN;
			}
			break;
		case PLACE_ARM_IN: /*The arm should rotate back to 0°*/
			if(!kfs_up.is_moving)
			{
				/*---IMPLEMENTATION FOR ROTATION---*/
//				Path_Profile_Set(&j60_prof,
//						current_pos_j60,
//						YAW_AD,
//						KFS_YAW_0 - current_pos_j60,
//						YAW_DD,
//						YAW_V);
				Ext_MoveTo(&kfs_grip, KFS_G_MAX);
				KFS.place_state = PLACE_End;
			}
			break;
		case PLACE_End: /*Move back to neutral position (Max Height, Closed Grip, Intake Off) */
			if(!kfs_up.is_moving && !kfs_grip.is_moving && (j60_prof.state == 0) && !front_ext.is_moving)
			{
				Ext.cmd = EXT_CMD_IDLE;
				KFS.place_state = PLACE_IDLE;
				KFS.cmd = KFS_CMD_IDLE;
			}
			break;
		default:
			break;
		}

	}
}
uint8_t mf_imu = 1;
void Ext_Task(void *argument){

	//	MODNRobotBaseVelInit(MODN_MECANUM, 1.0, 1.0,&modn_ext);
	//	MODNRobotConInit(&ext_x_vel, &ext_y_vel, &ext_w_vel, &modn_ext);
	//	MODNWheelVelInit(&ext_FR_vel,&ext_BR_vel,&ext_FL_vel,&ext_BL_vel,&modn_ext);
	Ext_Init(&front_ext,FRONT_EXT_POS);
	Ext_Init(&rear_ext, REAR_EXT_POS);
	Ext_SetMotionParams(&front_ext, 8.5, 1.2, 1.2);
	Ext_SetMotionParams(&rear_ext, 8.5, 1.2, 1.2);
	Ext_SetTolerance(&front_ext, 0.08);
	Ext_SetTolerance(&rear_ext, 0.08);
	//	uint8_t ramp_f = 0;
	uint8_t ext_f = 0;
	while(1)
	{



		if(ps4.button == CIRCLE)
		{
			while(ps4.button == CIRCLE);
			if(ext_f)
			{
				Ext_SetMotionParams(&front_ext, 6.5, 1.2, 1.2);
				Ext_SetMotionParams(&rear_ext, 6.5, 1.2, 1.2);
				Ext_MoveTo(&front_ext, Ext_conv(47.0));
				Ext_MoveTo(&rear_ext,  -Ext_conv(47.0));
				ext_f = 0;
			}
			else{
				Ext_SetMotionParams(&front_ext, 8.5, 1.2, 1.2);
				Ext_SetMotionParams(&rear_ext, 8.5, 1.2, 1.2);
				Ext_MoveTo(&front_ext, Ext_conv(22.0));
				Ext_MoveTo(&rear_ext,  -Ext_conv(22.0));
				ext_f = 1;
			}
			FExt_state = 1;
			BExt_state = 1;


		}
		if(ps4.button == SQUARE)
		{
			while(ps4.button == SQUARE);
			Ext_SetMotionParams(&front_ext, 8.0, 1.5, 1.5);
			Ext_SetMotionParams(&rear_ext, 8.0, 1.5, 1.5);
			Ext_MoveTo(&front_ext, Ext_conv(-1.0));
			Ext_MoveTo(&rear_ext,  -Ext_conv(-1.0));
			FExt_state = 0;
			BExt_state = 0;
		}
		if(ps4.button == TRIANGLE)
		{
			while(ps4.button == TRIANGLE);
			if(FExt_state)
			{
				Ext_SetMotionParams(&front_ext, 8.0, 1.5, 1.5);
				Ext_MoveTo(&front_ext, Ext_conv(-1.0));
				FExt_state = 0;
			}
			else
			{
				Ext_SetMotionParams(&front_ext, 8.5, 1.2, 1.2);
				Ext_MoveTo(&front_ext, Ext_conv(22.0));
				FExt_state = 1;
			}
		}

		if(ps4.button == CROSS)
		{
			while(ps4.button == CROSS);
			if(BExt_state)
			{
				Ext_SetMotionParams(&rear_ext, 8.0, 1.5, 1.5);
				Ext_MoveTo(&rear_ext, -Ext_conv(-1.0));
				BExt_state = 0;
			}
			else
			{
				Ext_SetMotionParams(&rear_ext, 8.5, 1.2, 1.2);
				Ext_MoveTo(&rear_ext, -Ext_conv(22.0));
				BExt_state = 1;
			}

		}
		//		if(ps4.button == RIGHT)
		//		{
		//			while(ps4.button == RIGHT);
		//			Ext_SetMotionParams(&front_ext, 6.5, 1.2, 1.2);
		//			Ext_SetMotionParams(&rear_ext, 6.5, 1.2, 1.2);
		//			Ext_MoveTo(&front_ext, Ext_conv(10.0));
		//			Ext_MoveTo(&rear_ext,  -Ext_conv(10.0));
		//
		//			FExt_state = 1;
		//			BExt_state = 1;
		//		}
		if(ps4.button == UP)
		{
			while(ps4.button == UP);
			if(KFS.cmd == KFS_CMD_IDLE){
				Ext.cmd = EXT_CMD_FORWARD;
				Ext.level = EXT_LEVEL_20;
				navi_state = NAVI_MF;
			}
		}
		if(ps4.button == R3)
		{
			while(ps4.button == R3);
			if(KFS.cmd == KFS_CMD_IDLE){
				Ext.cmd = EXT_CMD_FORWARD;
				Ext.level = EXT_LEVEL_40;
				navi_state = NAVI_MF;
			}
		}
		//		if(ps4.button == LEFT)
		//		{
		//			while(ps4.button == LEFT);
		//			if(KFS.cmd == KFS_CMD_IDLE){
		//			Ext.cmd = EXT_CMD_FORWARD;
		//			Ext.level = EXT_LEVEL_20;
		//			navi_state = NAVI_MF;
		//			}
		//		}

		//		typedef enum{
		//			EXT_LEVEL_20,
		//			EXT_LEVEL_40,
		//			EXT_LEVEL_MID,
		//			EXT_LEVEL_TOP
		//		}Ext_Level_t;
		//
		//		typedef enum{
		//			EXT_CMD_AUTOPICK,
		//			EXT_CMD_FORWARD,
		//			EXT_CMD_BACKWARD,
		//			EXT_CMD_LEFT,
		//			EXT_CMD_RIGHT,
		//			EXT_CMD_RAMP,
		//			EXT_CMD_BUSY,
		//			EXT_CMD_IDLE
		//		}Ext_Cmd_t;
		switch(Ext.cmd)
		{
		case EXT_CMD_AUTOPICK:
			break;
		case EXT_CMD_FORWARD:
			//			switch(KFS.direction)
			//			{
			//			case EXT_DIR_LEFT:
			//				tangle = 90.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case EXT_DIR_FORWARD:
			//				tangle = 0.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case EXT_DIR_RIGHT:
			//				tangle = -90.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			case EXT_DIR_BACK:
			//				tangle = -180.0;
			//				angle_error = 0;
			//				PIDDelayInit(&pid_z);
			//				break;
			//			default:
			//				break;
			//
			//			}
			osDelay(100);
			tVx = 0.15;
			mf_imu = 1;
			Ext.cmd = EXT_CMD_CONFIRMING;
			break;
		case EXT_CMD_RAMP:
			break;
		case EXT_CMD_CONFIRMING:
			if(readbit(F_FWRD) && readbit(DF_20))
			{
				tVx = 0;
				Ext.climb_state = CLIMB_Start;
				Ext.descend_state = DESCEND_IDLE;
				Ext.cmd = EXT_CMD_BUSY;
			}
			else if(!readbit(F_FWRD) && !readbit(DF_20))
			{
				tVx = 0;
				Ext.cmd = EXT_CMD_BUSY;
				Ext.climb_state = CLIMB_IDLE;
				Ext.descend_state = DESCEND_Start;
			}
			break;
		default:
			break;
		}

		switch(Ext.climb_state)
		{
		case CLIMB_IDLE:
			break;
		case CLIMB_Start: /*The extensions should be inside the robot by 1 cm*/
			Ext_MoveTo(&front_ext, Ext_conv(-0.4));
			Ext_MoveTo(&rear_ext,-Ext_conv(-0.4));
			navi_state = NAVI_MF;
			Ext.climb_state = CLIMB_UP;
			break;
		case CLIMB_UP: /*Extend up until the IR sensor at the front doesn't detect anymore and go to the nearest height (200 or 400)*/
			if(!front_ext.is_moving && !rear_ext.is_moving)
			{
				Ext.climb_state = CLIMB_FORWARD;
				switch(Ext.level)
				{
				case EXT_LEVEL_20:
					Ext_MoveTo(&front_ext, Ext_conv(22.0));
					Ext_MoveTo(&rear_ext,-Ext_conv(22.0));
					FExt_state = 1;
					BExt_state = 1;
					break;
				case EXT_LEVEL_40:
					Ext_MoveTo(&front_ext, Ext_conv(42.0));
					Ext_MoveTo(&rear_ext,-Ext_conv(42.0));
					FExt_state = 1;
					BExt_state = 1;
					break;
				default:
					Ext.climb_state = CLIMB_ERR;
					break;
				}
			}
			break;
		case CLIMB_FORWARD:/* Go forward with the extension wheels until the IR sensor
		 	 	 	 		  right infront of the front extension detects the block surface*/
			if(!front_ext.is_moving && !rear_ext.is_moving)
			{
				RBMS_Set_Target_Velocity(BExt_RW,  220);
				RBMS_Set_Target_Velocity(BExt_LW, -220);
				RBMS_Set_Target_Velocity(FExt_RW,  220);
				RBMS_Set_Target_Velocity(FExt_LW, -220);
				Ext.climb_state = CLIMB_RETRACT_FRONT;

			}

			break;
		case CLIMB_RETRACT_FRONT: /*Retract the front extensions*/
			if(readbit(UP_F))
			{
				RBMS_Set_Target_Velocity(FExt_LW,0);
				RBMS_Set_Target_Velocity(FExt_RW,0);
				RBMS_Set_Target_Velocity(BExt_LW,0);
				RBMS_Set_Target_Velocity(BExt_RW,0);
				Ext_MoveTo(&front_ext, Ext_conv(-1.0));
				FExt_state = 0;
				Ext.climb_state = CLIMB_INTO_BLOCK;
			}
			break;
		case CLIMB_INTO_BLOCK: /*Go forward with the extension wheels until the IR sensor
 	 	 	 	 	 		     right infront of the rear extensions detects the block surfcace*/
			if(!front_ext.is_moving)
			{
				tVx = 0.1;
				mf_imu = 0;
				RBMS_Set_Target_Velocity(BExt_RW,  180);
				RBMS_Set_Target_Velocity(BExt_LW, -180);
				Ext.climb_state = CLIMB_RETRACT_BACK;
			}
			break;
		case CLIMB_RETRACT_BACK: /*Retract the rear extensions*/
			if(readbit(UP_B))
			{
				tVx = 0.0;
				RBMS_Set_Target_Velocity(BExt_LW,0);
				RBMS_Set_Target_Velocity(BExt_RW,0);
				Ext_MoveTo(&rear_ext,-Ext_conv(-1.0));
				BExt_state = 0;
				Ext.climb_state = CLIMB_CLEAR_BLOCK;
			}
			break;
		case CLIMB_CLEAR_BLOCK: /*Move forward until the IR sensor at the back of the robot detects the block surface*/
			if(!rear_ext.is_moving)
			{
				tVx = 0.2;
				mf_imu = 1;
				Ext.timer = timer;
				Ext.climb_state = CLIMB_End;
			}
			break;
		case CLIMB_End:
			if(readbit(DOWN_B) && (timer - Ext.timer > 1.5))
			{
				tVx = 0;
				Ext.climb_state = CLIMB_IDLE;
				Ext.cmd = EXT_CMD_IDLE;
				navi_state = MANUAL;
				if(Ext.increment)
				{
					PathApp.step_status = APP_DONE;
				}
			}
			break;
		default:
			break;
		}

		switch(Ext.descend_state)
		{
		case DESCEND_IDLE:
			break;
		case DESCEND_Start: /*The extensions should be inside the robot by 1 cm*/
			Ext_MoveTo(&front_ext, Ext_conv(-0.4));
			Ext_MoveTo(&rear_ext,-Ext_conv(-0.4));
			navi_state = NAVI_MF;
			Ext.descend_state = DESCEND_EXTEND_FRONT;
			break;
		case DESCEND_EXTEND_FRONT: /*Extend the front extensions based on the two IR sensors right behind the front extensions*/
			if(!front_ext.is_moving && !rear_ext.is_moving)
			{
				if(Ext.level == EXT_LEVEL_20)
				{
					Ext_MoveTo(&front_ext, Ext_conv(22.0));
					FExt_state = 1;
					Ext.descend_state = DESCEND_FORWARD;
				}
				else if(Ext.level == EXT_LEVEL_40)
				{
					Ext_MoveTo(&front_ext, Ext_conv(42.0));
					FExt_state = 1;
					Ext.descend_state = DESCEND_FORWARD;
				}
				else{
					Ext.descend_state = DESCEND_ERR;
				}
			}
			break;
		case DESCEND_FORWARD: /*Move the extension wheels as well as the swerve forward
 	 	 	 	 	 	 	   until the IR sensor right behind the rear extensions stops detecting*/
			if(!front_ext.is_moving)
			{
				tVx = 0.1;
				RBMS_Set_Target_Velocity(FExt_RW, 130);
				RBMS_Set_Target_Velocity(FExt_LW,-130);
				mf_imu = 0;
				Ext.descend_state = DESCEND_STOP;
			}
			break;
		case DESCEND_STOP: /*Stop moving*/
			if(!readbit(DOWN_B))
			{
				tVx = 0;
				RBMS_Set_Target_Velocity(FExt_RW, 0);
				RBMS_Set_Target_Velocity(FExt_LW, 0);
				Ext.descend_state = DESCEND_EXTEND_REAR;
			}
			break;
		case DESCEND_EXTEND_REAR: /*Extend the rear extensions to match the length of the front extensions*/
			Ext.descend_state = DESCEND_CLEAR;
			if(Ext.level == EXT_LEVEL_20)
			{
				Ext_MoveTo(&rear_ext, -Ext_conv(22.0));
				BExt_state = 1;
			}
			else if(Ext.level == EXT_LEVEL_40)
			{
				Ext_MoveTo(&rear_ext, -Ext_conv(42.0));
				BExt_state = 1;

			}
			else{
				Ext.descend_state = DESCEND_ERR;
			}
			break;
		case DESCEND_CLEAR: /*Move the extensions wheels forward until the IR sensor the back of the robot stops detecting*/
			if(!rear_ext.is_moving)
			{
				RBMS_Set_Target_Velocity(FExt_RW, 200);
				RBMS_Set_Target_Velocity(FExt_LW,-200);
				RBMS_Set_Target_Velocity(BExt_RW, 200);
				RBMS_Set_Target_Velocity(BExt_LW,-200);
				Ext.descend_state = DESCEND_RETRACT_ALL;
				Ext.timer = timer;
			}
			break;
		case DESCEND_RETRACT_ALL: /*Retract Both the front and rear extensions*/
			if(timer - Ext.timer > 1.0)
			{
				RBMS_Set_Target_Velocity(FExt_RW, 0);
				RBMS_Set_Target_Velocity(FExt_LW, 0);
				RBMS_Set_Target_Velocity(BExt_RW, 0);
				RBMS_Set_Target_Velocity(BExt_LW, 0);
				Ext_MoveTo(&front_ext,  Ext_conv(-1.0));
				Ext_MoveTo(&rear_ext, -Ext_conv(-1.0));
				FExt_state = 0;
				BExt_state = 0;
				mf_imu = 1;
				Ext.descend_state = DESCEND_End;
			}
			break;
		case DESCEND_End: /*Set the state and the cmd back to IDLE*/
			if(!front_ext.is_moving && !rear_ext.is_moving)
			{
				Ext.cmd = EXT_CMD_IDLE;
				Ext.descend_state = DESCEND_IDLE;
				if(Ext.increment)
				{
					PathApp.step_status = APP_DONE;
				}
				navi_state = MANUAL;
			}
			break;
		default:
			break;
		}
	}
}

void MF_Task(void* argument)
{
	enum{
		MF_START = 1,
		MF_STOP,
		MF_UP,
		MF_DOWN,
		F_UP,
		F_FORWARD,
		F_FExt,
		F_PROCEED,
		F_BExt,
		F_END,
		B_FORWARD,
		B_FExt,
		B_PROCEED,
		B_BExt,
		B_END,
		MF_20,
		MF_40,
		MF_IDLE
	};
	//	uint8_t MF_State = 0;
	//	uint8_t Ext_L = 0;
	//	float MF_timer = 0.0;
	/* Sensors ill be using
	 * 1 at the front of the robot to detect the forest block infront of the robot
	 * 1 facing down at the front of the robot to detect if there is ground under the robot ( might be optional/ just replaced with the sensor below it)
	 * 1 facing down right behind the front swerve
	 * 1 facing down infront of the back extensions
	 * 1 or 2 facing down right behind the front extensions
	 * 1 facing down right behind the back extensions
	 *
	 * */
	while(1)
	{
		//		if(ps4.button == UP)
		//		{
		//			while(ps4.button == UP);
		//			MF_State = MF_START;
		//			navi_state = NAVI_MF;
		//		}
		//		switch(MF_State)
		//		{
		//		case MF_START:
		//			if(readbit(F_FWRD) && readbit(UP_F))
		//			{
		//				tVx = 0;
		//				Ext_MoveTo(&front_ext, Ext_conv(41.0));
		//				Ext_MoveTo(&rear_ext,  -Ext_conv(41.0));
		//				MF_State = F_UP;
		//				FExt_state = 1;
		//				BExt_state = 1;
		//
		//			}
		//			else if(!readbit(F_FWRD) && !readbit(UP_F))
		//			{
		//				tVx = 0.25;
		//				MF_State = B_FORWARD;
		//			}
		//			else
		//			{
		//				tVx = 0.15;
		//			}
		//			break;
		//		case F_UP:
		//			tVx = 0;
		//			while(readbit(F_FWRD));
		//			if(rbms3.motor[0].pos <= Ext_conv(30.0))
		//			{
		//				Ext_MoveTo(&front_ext, Ext_conv(22.0));
		//				Ext_MoveTo(&rear_ext,  -Ext_conv(22.0));
		//				FExt_state = 1;
		//				BExt_state = 1;
		//			}
		//			else
		//			{
		//				Ext_MoveTo(&front_ext, Ext_conv(41.0));
		//				Ext_MoveTo(&rear_ext,  -Ext_conv(41.0));
		//				FExt_state = 1;
		//				BExt_state = 1;
		//			}
		//			while(front_ext.is_moving);
		//			MF_State = F_FORWARD;
		//			break;
		//		case F_FORWARD:
		//			RBMS_Set_Target_Velocity(BExt_RW, 200);
		//			RBMS_Set_Target_Velocity(BExt_LW, -200);
		//			if(readbit(UP_F))
		//			{
		//				RBMS_Set_Target_Velocity(BExt_RW, 0);
		//				RBMS_Set_Target_Velocity(BExt_LW, 0);
		//				Ext_MoveTo(&front_ext, Ext_conv(-0.5));
		//				FExt_state = 0;
		//				MF_State = F_FExt;
		//			}
		//			break;
		//		case F_FExt:
		//			if(!front_ext.is_moving)
		//			{
		//				MF_State = F_PROCEED;
		//			}
		//			break;
		//		case F_PROCEED:
		//			tVy = 0.1;
		//			RBMS_Set_Target_Velocity(BExt_RW, 300);
		//			RBMS_Set_Target_Velocity(BExt_LW, -300);
		//			if(readbit(UP_B))
		//			{
		//				RBMS_Set_Target_Velocity(BExt_RW, 0);
		//				RBMS_Set_Target_Velocity(BExt_LW, 0);
		//				tVy = 0;
		//				Ext_MoveTo(&rear_ext,  -Ext_conv(-0.5));
		//				BExt_state = 0;
		//				MF_State = F_BExt;
		//			}
		//			break;
		//		case F_BExt:
		//			if(!(rear_ext.is_moving))
		//			{
		//				tVy = 0.15;
		//				MF_timer = timer;
		//				MF_State = F_END;
		//			}
		//			break;
		//		case F_END:
		//			if(timer - MF_timer > 1)
		//			{
		//				tVy = 0;
		//				tVx = 0;
		//				navi_state = MANUAL;
		//				MF_State = MF_IDLE;
		//			}
		//			break;
		//		case B_FORWARD:
		//			if(!readbit(DF_20))
		//			{
		//				tVx = 0;
		//				tVy = 0;
		//
		//				if(readbit(DF_40))
		//				{
		//					Ext_MoveTo(&front_ext, Ext_conv(22.0));
		//					Ext_L = MF_20;
		//				}
		//				else
		//				{
		//					Ext_MoveTo(&front_ext, Ext_conv(41.0));
		//					Ext_L = MF_40;
		//
		//				}
		//				MF_State = B_FExt;
		//				FExt_state = 1;
		//			}
		//			break;
		//		case B_FExt:
		//			if(!front_ext.is_moving){
		//				MF_State = B_PROCEED;
		//			}
		//			break;
		//		case B_PROCEED:
		//			tVx = 0.2;
		//			if(!readbit(DOWN_B))
		//			{
		//				tVx = 0;
		//				if(Ext_L == MF_40)
		//					Ext_MoveTo(&front_ext, Ext_conv(41.0));
		//				else
		//					Ext_MoveTo(&front_ext, Ext_conv(22.0));
		//				MF_State = B_BExt;
		//				BExt_state = 1;
		//
		//			}
		//			break;
		//		case B_BExt:
		//			tVx = 0;
		//			if(!rear_ext.is_moving)
		//			{
		//				MF_timer = timer;
		//				tVx = 0;
		//				RBMS_Set_Target_Velocity(BExt_RW, 200);
		//				RBMS_Set_Target_Velocity(BExt_LW, -200);
		//				MF_State = B_END;
		//			}
		//			break;
		//		case B_END:
		//			if(timer - MF_timer > 0.75)
		//			{
		//				RBMS_Set_Target_Velocity(BExt_RW, 0);
		//				RBMS_Set_Target_Velocity(BExt_LW, 0);
		//				tVx = 0;
		//				tVy = 0;
		//				navi_state = MANUAL;
		//				MF_State = MF_IDLE;
		//			}
		//			break;
		//		}
		osDelay(100);
	}
}


float clamp(float min,float value,float max){if(value>=max){return max;}else if(value<=min){return min;}else{return value;}}


//void velramp(vel_cmd_t* current, vel_cmd_t* target) {
//    // 1. Define Step Sizes (Acceleration limits)
//    // Adjust these based on your loop frequency
//    const float step_xy = 2.0 * 0.005f;
//    const float step_w  = 0.005f;
//
//    // 2. Calculate Vector Difference for X and Y
//    float delta_x = target->x - current->x;
//    float delta_y = target->y - current->y;
//
//    // Calculate Euclidean distance in velocity space
//    float distance = sqrtf(delta_x * delta_x + delta_y * delta_y);
//
//    // 3. Apply Vector Ramping (Maintains Direction)
//    if (distance > step_xy && distance > 0.0001f) {
//        // Move exactly 'step_xy' distance toward target
//        float scale = step_xy / distance;
//        current->x += delta_x * scale;
//        current->y += delta_y * scale;
//    } else {
//        // Close enough to target, snap to it
//        current->x = target->x;
//        current->y = target->y;
//    }
//
//    // 4. Apply Linear Ramping for Angular Velocity (W)
//    float delta_w = target->w - current->w;
//    if (fabsf(delta_w) > step_w) {
//        if (delta_w > 0) current->w += step_w;
//        else            current->w -= step_w;
//    } else {
//        current->w = target->w;
//    }
//}

void velramp() {
	stepx = 7.0 * 0.005;
	stepy = 7.0 * 0.005;

	stepw = 6.0 * 0.005;
	float step = fmin(stepx, stepy);  // use the smaller step to ensure smoothness

	float delta_x = tVx - xRvel;
	float delta_y = tVy - yRvel;

	float distance = sqrt(delta_x * delta_x + delta_y * delta_y);

	if (distance > step) {
		// Normalize the delta and apply step size proportionally
		float scale = step / distance;
		xRvel += delta_x * scale;
		yRvel += delta_y * scale;
	} else {
		// Close enough — snap directly to target
		xRvel = tVx;
		yRvel = tVy;
	}

	Vx = xRvel;
	Vy = yRvel;

	//    if(fabs(delta_x) + fabs(delta_y) > 0.1){
	//    	x_vel = (fabs(xRvel) > 0.1) ? xRvel : 0;
	//		y_vel = (fabs(yRvel) > 0.1) ? yRvel : 0;
	//    }else{
	x_vel = xRvel;
	y_vel = yRvel;

	float delta_w = tVw - wRvel;

	if (fabs(delta_w) > stepw) {
		if (delta_w > 0)
			wRvel += stepw;
		else
			wRvel -= stepw;
	} else {
		wRvel = tVw;
	}

	w_vel = wRvel;
	//    }
}
//void NaviTask(void* arg)
//{
//	MODNRobotBaseVelInit(MODN_FWD_SWERVE, 0.75, 0.75, &modn);
//	MODNRobotConInit(&vel_cmd.x, &vel_cmd.y, &vel_cmd.w, &modn);
//	MODNWheelVelInit(&myswerve.vel[0],&myswerve.vel[2],&myswerve.vel[1],&myswerve.vel[3],&modn);
//	MODNWheelDirInit(&myswerve.ang[0],&myswerve.ang[2],&myswerve.ang[1],&myswerve.ang[3],&modn);
//
//	SwerveInit(MODN_FWD_SWERVE,2.0f,0.75,0.75,&vesc,&rbms1,&myswerve);
//
//	while(1)
//	{
//		angle_error=tangle-IMU.real_z;
//		PID(&pid_z);
//		if(fabs(angle_error) < 1.0){
//			angle_output = 0;
//		}
//		vel_cmd.w= -(angle_output);
//		switch(navi_state)
//		{
//		case MANUAL:
//			vel_cmd.x=1.5*(ps4.joyL_x*cos(IMU.real_zrad + (M_PI/2)) + ps4.joyL_y*sin(IMU.real_zrad + (M_PI/2)));
//			vel_cmd.y=1.5*(ps4.joyL_x*sin(IMU.real_zrad + (M_PI/2)) - ps4.joyL_y*cos(IMU.real_zrad + (M_PI/2)));
//
//			if(fabs(ps4.joyR_2-ps4.joyL_2) > 0.05 )
//			{
//				vel_cmd.w= - (ps4.joyR_2-ps4.joyL_2);
//				tangle = IMU.real_z;
//				angle_error = 0;
//				PIDDelayInit(&pid_z);
//			}
//			break;
//		case NAVI_PP:
//			if(pathplan.mode == NAV_MODE_PATHPROFILE){
//				vel_cmd.x = pathplan.x_vel;
//				vel_cmd.y = pathplan.y_vel;
//			}
//			if(pathplan.mode == NAV_MODE_PURE_PID)
//			{
//				vel_cmd.x = pathplan.x_vel;
//				vel_cmd.y = pathplan.y_vel;
//				//tVw = -pathplan.w_vel;
//			}
//			//w_vel = pathplan.w_vel;
//			//tangle = IMU.real_z;
//			if(pathplan.completed)
//			{
//				//pathplan.mode = NAV_MODE_PURE_PID;
//				//pp_stopped = 1;
//			}
//			break;
//		case NAVI_MC:
//			if(mc_pp.mode == NAV_MODE_PATHPROFILE){
//				vel_cmd.x = mc_pp.x_vel;
//				vel_cmd.y = mc_pp.y_vel;
//			}
//			if(mc_pp.mode == NAV_MODE_PURE_PID)
//			{
//				vel_cmd.x = mc_pp.x_vel;
//				vel_cmd.y = mc_pp.y_vel;
//				//tVw = -pathplan.w_vel;
//			}
//			//w_vel = pathplan.w_vel;
//			//tangle = IMU.real_z;
//			if(mc_pp.completed)
//			{
//				//pathplan.mode = NAV_MODE_PURE_PID;
//				//pp_stopped = 1;
//			}
//			break;
//		case NAVI_MF:
//			break;
//		default:
//			vel_cmd.x =0;
//			vel_cmd.y =0;
//			vel_cmd.w =0;
//			break;
//		}
//		encoderpos();
//		MODNUpdate(&modn);
//	}
//}
void navi(){
	velramp();

	if(fabs(ps4.joyR_2-ps4.joyL_2) > 0.05 )
	{
		tVw= -1.5 * (ps4.joyR_2-ps4.joyL_2);
		tangle = IMU.real_z;
		angle_error = 0;
		PIDDelayInit(&pid_z);
	}
	else{
		angle_error=tangle-IMU.real_z;
		PID(&pid_z);
		if(fabs(angle_error) < 0.9){
			angle_output = 0;
		}
		w_vel = angle_output;

	}
	switch (navi_state){
	case MANUAL:
		tVx=  2.5*( ps4.joyL_y*cos(IMU.real_zrad ) + ps4.joyL_x*sin(IMU.real_zrad));
		tVy=  2.5*(-ps4.joyL_y*sin(IMU.real_zrad ) + ps4.joyL_x*cos(IMU.real_zrad));
		break;
	case NAVI_PP:
		if(path_plan.mode == NAV_MODE_PATHPROFILE){
			x_vel = path_plan.x_vel;
			y_vel = path_plan.y_vel;
		}
		if(path_plan.mode == NAV_MODE_PURE_PID)
		{
			tVx = path_plan.x_vel;
			tVy = path_plan.y_vel;
			//tVw = -pathplan.w_vel;
		}
		//w_vel = pathplan.w_vel;
		//tangle = IMU.real_z;
		if(path_plan.completed)
		{
			//pathplan.mode = NAV_MODE_PURE_PID;
			//pp_stopped = 1;
		}
		break;
	case NAVI_MC:
		if(mc_pp.mode == NAV_MODE_PATHPROFILE){
			x_vel = -mc_pp.x_vel*cos(IMU.real_zrad) - mc_pp.y_vel*sin(IMU.real_zrad);
			y_vel = -mc_pp.x_vel*sin(IMU.real_zrad) + mc_pp.y_vel*cos(IMU.real_zrad);
		}
		else if(mc_pp.mode == NAV_MODE_PURE_PID)
		{
			tVx = -mc_pp.x_vel * cos(IMU.real_zrad) - mc_pp.y_vel * sin(IMU.real_zrad);
			tVy =  -mc_pp.x_vel * sin(IMU.real_zrad) + mc_pp.y_vel * cos(IMU.real_zrad);
		}
		//		if(mc_fir.mode == NAVFIR_MODE_MOVE){
		//			x_vel = -mc_fir.x_vel*cos(IMU.real_zrad) - mc_fir.y_vel*sin(IMU.real_zrad);
		//			y_vel = -mc_fir.x_vel*sin(IMU.real_zrad) + mc_fir.y_vel*cos(IMU.real_zrad);
		//
		//		}
		//		if(mc_fir.mode == NAVFIR_MODE_PURE_PID)
		//		{
		//			tVx = mc_fir.x_vel * cos(IMU.real_zrad) + mc_fir.y_vel * sin(IMU.real_zrad);
		//			tVy = -mc_fir.x_vel* sin(IMU.real_zrad) + mc_fir.y_vel * cos(IMU.real_zrad);
		//
		//		}

		//		if(mc_pp.completed)
		//		{
		//			//pathplan.mode = NAV_MODE_PURE_PID;
		//			//pp_stopped = 1;
		//		}
		break;
	case NAVI_MF:
		if(!mf_imu){
			w_vel = 0;
		}
		break;
	case STOP:
		tVx = 0;
		tVy = 0;
		tVw = 0;
		break;
	default:
		break;

	}

	if((Ext.climb_state == CLIMB_IDLE) && (Ext.descend_state == DESCEND_IDLE))
	{
		Ext_WheelRun();
		//		ext_x_vel =  (ps4.joyR_y*cos(IMU.real_zrad ) - ps4.joyR_x*sin(IMU.real_zrad));
		//		ext_y_vel =  (ps4.joyR_y*sin(IMU.real_zrad ) + ps4.joyR_x*cos(IMU.real_zrad));
		//		if(fabs(ps4.joyR_2-ps4.joyL_2) > 0.05 ){
		//			ext_w_vel =	 -(ps4.joyR_2-ps4.joyL_2);
		//		}
		//		else{
		//			ext_w_vel = angle_output;
		//		}

	}
	MODNUpdate(&modn_ext);
	MODNUpdate(&modn);
	SwerveRun(&myswerve);

}
void Ext_WheelRun(){
	RBMS_Set_Target_Velocity(FExt_RW,  300 * ps4.joyR_y);
	RBMS_Set_Target_Velocity(BExt_RW,  300 * ps4.joyR_y);
	RBMS_Set_Target_Velocity(FExt_LW, -300 * ps4.joyR_y);
	RBMS_Set_Target_Velocity(BExt_LW, -300 * ps4.joyR_y);
}
void encoderpos() //call in a 5ms interrupt
{
	now_theta = IMU.real_zrad - z_offset * M_PI / 180.0;
	delta_theta = now_theta - prev_theta;

	while (delta_theta > M_PI) delta_theta -= 2 * M_PI;
	while (delta_theta < -M_PI) delta_theta += 2 * M_PI;

	float avg_theta = prev_theta + (delta_theta / 2.0f);

	dx_raw = -H7_QEI_read(&QEI1) - raw_x;
	dy_raw = H7_QEI_read(&QEI2) - raw_y;

	//Compensate for rotation (encoder offset) //in meters
	rot_x = 0.00; // back (front) offset of X encoder (in Y-axis)
	rot_y = -0.254; // left (right) offset of Y encoder (in X-axis)

	dx_rot =  delta_theta * rot_x * 8192.0 / (0.06 * M_PI); // convert to ticks
	dy_rot =  delta_theta * rot_y * 8192.0 / (0.06 * M_PI);

	//Subtract rotational effect
	dx = dx_raw - dx_rot;
	dy = dy_raw - dy_rot;

	actual_x += ((dx * cos(avg_theta) + dy * sin(avg_theta)) * 0.06 * M_PI / (8192.0));
	actual_y += ((dy * cos(avg_theta) - dx * sin(avg_theta)) * 0.06 * M_PI / (8192.0));

	raw_x = -H7_QEI_read(&QEI1);
	raw_y = H7_QEI_read(&QEI2);
	prev_theta = now_theta;
}

void swervepos() //call in a 5ms interrupt
{
	// 1. Read angles (steering)
	radnow[0] = rbms1.motor[RBMS1].pos * -2 * M_PI;
	radnow[1] = rbms1.motor[RBMS2].pos * -2 * M_PI;
	radnow[2] = rbms1.motor[RBMS3].pos * -2 * M_PI;
	radnow[3] = rbms1.motor[RBMS4].pos * -2 * M_PI;

	// 2. Read drive encoders
	for (uint8_t i = 0; i < 4; i++) {
		tacnow[i] = infolist[i].tacho_value;
	}

	// 3. Read heading
	headingnow = IMU.real_zrad - real_z_offset * M_PI / 180.0;

	// 4. Compute delta ticks
	for (uint8_t i = 0; i < 4; i++) {
		delta_tac[i] = tacnow[i] - tacprev[i];
		tacprev[i] = tacnow[i];  // update for next loop
	}

	// 5. Calculate delta X, Y in robot frame
	for (uint8_t i = 0; i < 4; i++) {
		float d = delta_tac[i];  // delta movement in ticks
		dx_swerve += d * cosf(radnow[i]);
		dy_swerve += d * sinf(radnow[i]);
	}

	dx_swerve /= 4.0f;
	dy_swerve /= 4.0f;

	// 6. Convert robot frame movement to world frame
	real_dx_swerve = dx_swerve * sinf(headingnow) + dy_swerve * cosf(headingnow);
	real_dy_swerve = dx_swerve * cosf(headingnow) - dy_swerve * sinf(headingnow);

	// 7. Integrate position (still in ticks)
	x_swerve += real_dx_swerve / 7.46; //prev constant is 9.87
	y_swerve += real_dy_swerve / 7.46;
}

static int parse_float_strict(const char *token, float *out)
{
	if (!token || !*token) return 0;

	char *endptr;
	errno = 0;
	float val = strtof(token, &endptr);

	/* no characters consumed => invalid */
	if (endptr == token) return 0;

	/* allow trailing spaces, but nothing else */
	while (*endptr && isspace((unsigned char)*endptr)) endptr++;

	if (*endptr != '\0') return 0; /* garbage after number */

	/* optional: you can also check errno for overflow (ERANGE) if you want */
	*out = val;
	return 1;
}

void Parse3Floats(char *str)
{
	if (!str) return;

	char *saveptr;
	char *token;
	int type = -1;
	float f1 = 0.0f, f2 = 0.0f, f3 = 0.0f;

	/* header */
	token = strtok_r(str, ",", &saveptr);
	if (!token) return;

	if      (strcmp(token, "XY")  == 0) type = 0;
	else if (strcmp(token, "W")   == 0) type = 1;
	else if (strcmp(token, "IMU") == 0) type = 2;
	else if (strcmp(token, "X")	  == 0) type = 3;
	else if (strcmp(token, "P")	  == 0) type = 4;
	else if	(strcmp(token, "S")   == 0) type = 5;
	else if (strcmp(token, "V")   == 0) type = 6;
	else return; /* unknown header */

	/* first float */
	token = strtok_r(NULL, ",", &saveptr);
	if (!parse_float_strict(token, &f1)) return;

	/* second float */
	token = strtok_r(NULL, ",", &saveptr);
	if (!parse_float_strict(token, &f2)) return;

	/* third float */
	token = strtok_r(NULL, ",", &saveptr);
	if (!parse_float_strict(token, &f3)) return;

	switch (type)
	{
	case 0: /* XY */
		//		mc_kp = f1;
		////		mc_ki = f2;
		//		mc_kd = f2;
		//		mc_pp.pid_x.K[KN] = f3;
		//		mc_pp.pid_y.K[KN] = f3;
		//		Navigator_TunePID_Position(&mc_pp, f1, f2, f3);
		break;

	case 1: /* W */
		Navigator_TunePID_Yaw(&path_plan, f1, f2, f3);
		break;

	case 2: /* IMU */
		PIDGainSet(KP, f1, &pid_z);
		PIDGainSet(KI, f2, &pid_z);
		PIDGainSet(KD, f3, &pid_z);
		break;
	case 3:
		mc_kp_x = f1;
		mc_kd_x = f2;
		mc_kn_x = f3;

		break;
	case 4:
		mc_pp.prof_x.k_val = f1;
		mc_pp.prof_y.k_val = f1;
		mc_pp.prof_x.start_t = f2;
		mc_pp.prof_y.start_t = f2;
		mc_pp.prof_x.end_t = f3;
		mc_pp.prof_y.end_t = f3;

		break;
	case 5:
		spear_point[1].x = f1;
		spear_point[1].y = f2;
		break;
	case 6:
		mc_vel = f1;
		mc_accel_d = f2;
		mc_decel_d = f3;
	}
}


float round_mf(float input)
{
	if(fabs(input) > Ext_conv(19.0) && fabs(input) < Ext_conv (35.0) )
	{
		return 21.5;
	}
	else{
		return 40.0;
	}
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
  while (1)
  {
	  led6 = 1;
  }
  /* USER CODE END Error_Handler_Debug */
}




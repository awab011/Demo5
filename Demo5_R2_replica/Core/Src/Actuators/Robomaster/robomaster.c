/*
 * robomaster.c
 *
 *  Created on: Nov 27, 2025
 *      Author: mua
 */


/*
 * Robomaster.c
 *
 *  Created on: 25 Jul 2023
 *      Author: KZ
 */

#include "../../Actuators/Robomaster/robomaster.h"

#include "../../adapter.h"
#define ABS(x)	((x>0) ? x : -x)

RBMS_t rbms1, rbms2, rbms3, rbms4, rbms5, rbms6;

/*	BEFORE START, MAKE SURE:
 * 	1) Set the Controller ID for the controller used (refer to documentation).
 * 	2) Set the Controller feedback frequency to 1kHz via robomaster assistant.
 * 	3) Configure mainboard CAN baud rate (set to 1MHz (actually is 1Mbps)).
 * 	4) Do not mix lower Controller ID (1234) with upper Controller ID (5678) in the same RBMS_t *rbms.
 */

/*
 * Function Name		: RBMS_Init
 * Function Description : Called to init Robomaster speed controller.
 * Function Remarks		: Init Robomaster speed controller with respective CAN handle and CAN ID.
 * Function Arguments	: *rbms			,	pointer to structure RBMS_t
 * 						  *fdcan			,	pointer to CAN_HandleTypeDef
 * 						  instance		,	Robomaster CAN ID (enumeration)
 * Function Return		: None
 * Function Example		: RBMS_Init(&rbms1, &fdcan1, RBMS1);
 */
void RBMS_Init(RBMS_t *rbms, FDCAN_HandleTypeDef *hfdcan, RBMS_Instance_t instance) {
	rbms->Init = 1;
	rbms->PID_Init = 0;
	rbms->Instance = instance;
	rbms->hfdcan = hfdcan;
	rbms->pTxMsg.IdType = FDCAN_STANDARD_ID;
	rbms->pTxMsg.Identifier = instance;
	rbms->pTxMsg.TxFrameType = FDCAN_DATA_FRAME;
	rbms->pTxMsg.DataLength = FDCAN_DLC_BYTES_8;
	int i = 0;
	while (i < 4) {
		RBMS_Config(rbms, i, 0, 0);
		i++;
	}
}

/*
 * Function Name		: RBMS_Config
 * Function Description : Called to config Robomaster speed controller.
 * Function Remarks		: Init Robomaster speed controller with respective motor number.
 * Function Arguments	: *rbms			,	pointer to structure RBMS_t
 * 						  num			,	motor number (0 to 3)
 * 						  type			,	Robomaster Speed Controller Type (enumeration)
 * 						  gear_ratio	,	gear ratio of motor
 * 						  wheel_diameter,	wheel diameter of motor
 * Function Return		: None
 * Function Example		: RBMS_Config(&rbms1, RBMS1, C610, 1.0, 0.05);
 */
void RBMS_Config(RBMS_t *rbms, RBMS_Num_t num, RBMS_Controller_t type,
		float gear_ratio) {
	rbms->motor[num].config.Controller = type;
	rbms->motor[num].config.gear_ratio = gear_ratio;
	rbms->motor[num].config.wheel_diameter = 0.1;
	rbms->motor[num].delta_pos = 0;
	rbms->motor[num].cycle_cnt = 0;
	rbms->motor[num].prev_raw_pos = 0;
	rbms->motor[num].rotor_pos = 0;
	rbms->motor[num].shaft_pos = 0;
	rbms->motor[num].reset_pos = 1;
	rbms->motor[num].offset_pos = 0;
	rbms->motor[num].control_mode = IDLE;

	switch (rbms->motor[num].config.Controller) {
	case C610:
		rbms->motor[num].input_gradient = RBMS_C610_INPUT_GRADIENT;
		rbms->motor[num].max_input = RBMS_C610_MAX_INPUT;
		rbms->motor[num].type_gear_ratio = RBMS_C610_GEAR_RATIO;
		rbms->motor[num].config.POS_P = 150;
		rbms->motor[num].config.POS_I = 0.03;
		rbms->motor[num].config.POS_D = 0;
		rbms->motor[num].config.VEL_P = 8;
		rbms->motor[num].config.VEL_I = 0.03;
		rbms->motor[num].config.VEL_D = 0;
		rbms->motor[num].config.pos_limit = 300;
		rbms->motor[num].config.vel_limit = 500;//0-600
		rbms->motor[num].config.cur_limit = 10;//10 max
		break;
	case C620:
		rbms->motor[num].input_gradient = RBMS_C620_INPUT_GRADIENT;
		rbms->motor[num].max_input = RBMS_C620_MAX_INPUT;
		rbms->motor[num].type_gear_ratio = RBMS_C620_GEAR_RATIO;
		rbms->motor[num].config.POS_P = 150;
		rbms->motor[num].config.POS_I = 0.02;
		rbms->motor[num].config.POS_D = 0;
		rbms->motor[num].config.VEL_P = 8;
		rbms->motor[num].config.VEL_I = 0.03;
		rbms->motor[num].config.VEL_D = 0;
		rbms->motor[num].config.pos_limit = 100;
		rbms->motor[num].config.vel_limit = 500;
		rbms->motor[num].config.cur_limit = 20;
		break;
	default:
		rbms->motor[num].input_gradient = 0;
		rbms->motor[num].max_input = 0;
		rbms->motor[num].config.POS_P = 0;
		rbms->motor[num].config.POS_I = 0;
		rbms->motor[num].config.POS_D = 0;
		rbms->motor[num].config.VEL_P = 0;
		rbms->motor[num].config.VEL_I = 0;
		rbms->motor[num].config.VEL_D = 0;
		rbms->motor[num].config.pos_limit = 100;
		rbms->motor[num].config.vel_limit = 200;
		rbms->motor[num].config.cur_limit = 5;
		break;
	}
}

/*
 * Function Name		: RBMS_PID_Init
 * Function Description : Called to init PID for Robomaster speed controller.
 * Function Remarks		: Init PID for each motor in RBMS_t *rbms
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * Function Return		: None
 * Function Example		: RBMS_PID_Init(&rbms1);
 */
void RBMS_PID_Init(RBMS_t *rbms){
	rbms->PID_Init = 1;
	for (int i=0; i<4; i++){
		if (rbms->motor[i].config.Controller != 0){
			PIDSourceInit(&rbms->motor[i].pos_error, &rbms->motor[i].t_rpm, &rbms->motor[i].P_PID);
			PIDGainInit(0.005, 1.0, 1.0/100, 600, rbms->motor[i].config.POS_P, rbms->motor[i].config.POS_I, rbms->motor[i].config.POS_D, 1, &rbms->motor[i].P_PID);
			PIDDelayInit(&rbms->motor[i].P_PID);
			PIDSourceInit(&rbms->motor[i].vel_error, &rbms->motor[i].t_current, &rbms->motor[i].V_PID);
			PIDGainInit(0.005, 1.0, 1.0/600, rbms->motor[i].config.cur_limit, rbms->motor[i].config.VEL_P*10/rbms->motor[i].config.cur_limit, rbms->motor[i].config.VEL_I, rbms->motor[i].config.VEL_D, 1, &rbms->motor[i].V_PID);
			PIDDelayInit(&rbms->motor[i].V_PID);
		}
	}
}

/*
 * Function Name		: RBMS_Set_Control_Mode
 * Function Description : Called to set the control mode for Robomaster speed controller.
 * Function Remarks		: Core function.
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * 						  num		,	motor number (0 to 3)
 * 						  mode		,	control mode (enumeration)
 * Function Return		: None
 * Function Example		: RBMS_Set_Control_Mode(&rbms1, RBMS1, POSITION);
 */
void RBMS_Set_Control_Mode(RBMS_t *rbms, RBMS_Num_t num, RBMS_Control_Mode_t mode){
	rbms->motor[num].control_mode = mode;
	rbms->motor[num].reset_pos = 1;
	rbms->motor[num].t_current = 0;
	rbms->motor[num].t_rpm = 0;
	rbms->motor[num].t_pos = 0;
}

/*
 * Function Name		: RBMS_Set_Target_Current
 * Function Description : Called to set the target current for current control.
 * Function Remarks		: Core function.motor
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * 						  num		,	motor number (0 to 3)
 * 						  current	,	target current (in Ampere)
 * Function Return		: None
 * Function Example		: RBMS_Set_Target_Current(&rbms1, RBMS1, 1.0);
 */
void RBMS_Set_Target_Current(RBMS_t *rbms, RBMS_Num_t num, float current){
	rbms->motor[num].t_current = current;
}

/*
 * Function Name		: RBMS_Set_Target_Velocity
 * Function Description : Called to set the target velocity for velocity control.
 * Function Remarks		: Core function.
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * 						  num		,	motor number (0 to 3)
 * 						  rpm		,	target rpm
 * Function Return		: None
 * Function Example		: RBMS_Set_Target_Velocity(&rbms1, RBMS1, 100);
 */
void RBMS_Set_Target_Velocity(RBMS_t *rbms, RBMS_Num_t num, float rpm){
	rbms->motor[num].t_rpm = rpm;
}

/*
 * Function Name		: RBMS_Set_Target_Position
 * Function Description : Called to set the target position for position control.
 * Function Remarks		: Core function.
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * 						  num		,	motor number (0 to 3)
 * 						  pos	,	target position (in rotation)
 * Function Return		: None
 * Function Example		: RBMS_Set_Target_Position(&rbms1, RBMS1, 1.0);
 */
void RBMS_Set_Target_Position(RBMS_t *rbms, RBMS_Num_t num, float pos){
	rbms->motor[num].t_pos = pos;
}

/*
 * Function Name		: RBMS_Velocity_Control
 * Function Description : Called to calculate PID for velocity control.
 * Function Remarks		: Core function.
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * 						  num			,	motor number (0 to 3)
 * Function Return		: None
 * Function Example		: RBMS_Velocity_Control(&rbms1);
 */
void RBMS_Velocity_Control(RBMS_t *rbms, RBMS_Num_t num){
	if (rbms->motor[num].config.Controller != 0){
		chlim(rbms->motor[num].t_rpm, rbms->motor[num].config.vel_limit);
		rbms->motor[num].vel_error = rbms->motor[num].t_rpm - rbms->motor[num].rpm;
		PID(&rbms->motor[num].V_PID);
		chlim(rbms->motor[num].t_current, rbms->motor[num].config.cur_limit);
	}
}

/*
 * Function Name		: RBMS_Position_Control
 * Function Description : Called to calculate PID for position control.
 * Function Remarks		: Core function.
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
* 						  num			,	motor number (0 to 3)
 * Function Return		: None
 * Function Example		: RBMS_Position_Control(&rbms1);
 */
void RBMS_Position_Control(RBMS_t *rbms, RBMS_Num_t num){
	if (rbms->motor[num].config.Controller != 0){
		chlim(rbms->motor[num].t_pos, rbms->motor[num].config.pos_limit);
		rbms->motor[num].t_shaft_pos = rbms->motor[num].t_pos * rbms->motor[num].config.gear_ratio;
		rbms->motor[num].pos_error = rbms->motor[num].t_shaft_pos - rbms->motor[num].shaft_pos;
		PID(&rbms->motor[num].P_PID);
	}
}

/*
 * Function Name		: RBMS_5ms
 * Function Description : Called to run the Robomaster speed controller at 5ms interval.
 * Function Remarks		: Use this function in FreeRTOS task with 5ms Semaphore.
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * Function Return		: None
 * Function Example		: RBMS_5ms(&rbms1);
 */
void RBMS_5ms(RBMS_t *rbms){
	for (int i=0; i<4; i++){
		switch (rbms->motor[i].control_mode){
		case POSITION:
			RBMS_Position_Control(rbms, i);
		case VELOCITY:
			RBMS_Velocity_Control(rbms, i);
			break;
		case CURRENT:
			break;
		case IDLE:
			rbms->motor[i].t_current = 0;
			break;
		}
	}
	RBMS_Send(rbms);
}

/*
 * Function Name		: RBMS_Send
 * Function Description : Called to send the data to Robomaster speed controller.
 * Function Remarks		: Core function.
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * Function Return		: None
 * Function Example		: RBMS_Send(&rbms1);
 */
void RBMS_Send(RBMS_t *rbms) {
	uint8_t data[8];
	int16_t current[4];

	for (int i = 0; i < 4; i++) {
		current[i] = (int16_t) (rbms->motor[i].t_current * rbms->motor[i].input_gradient);
		chlim(current[i], rbms->motor[i].max_input);

		data[i * 2] = (current[i] >> 8);
		data[i * 2 + 1] = current[i] & 0x00ff;
	}

	FDCAN_TxMsg(rbms->hfdcan, rbms->pTxMsg.Identifier, data, FDCAN_DLC_BYTES_8);

}

/*
 * Function Name		: RBMS_ACST_Calculation
 * Function Description : Called to calculate the data received from Robomaster speed controller.
 * Function Remarks		: None
 * Function Arguments	: *rbms		,	pointer to structure RBMS_t
 * Function Return		: None
 * Function Example		: RBMS_ACST_Calculation(&rbms1);
 */
void RBMS_ACST_Calculation(RBMS_t *rbms, RBMS_Num_t num) {
	rbms->motor[num].prev_raw_pos = rbms->motor[num].raw_pos;
	rbms->motor[num].raw_pos = (uint16_t) ((rbms->rxdata[0] << 8)
			+ rbms->rxdata[1]);
	rbms->motor[num].raw_rpm =
			(int16_t) ((rbms->rxdata[2] << 8) + rbms->rxdata[3]);
	rbms->motor[num].raw_current = (int16_t) ((rbms->rxdata[4] << 8)
			+ rbms->rxdata[5]);
	rbms->motor[num].temperature = rbms->rxdata[6];
	rbms->motor[num].delta_pos = (int16_t) rbms->motor[num].raw_pos
			- (int16_t) rbms->motor[num].prev_raw_pos;
	if (rbms->motor[num].reset_pos == 1){
		rbms->motor[num].offset_raw_pos = (int16_t) rbms->motor[num].raw_pos;
		rbms->motor[num].cycle_cnt = 0;
		rbms->motor[num].reset_pos = 0;
	} else {
		if (rbms->motor[num].delta_pos < -4000) {
			rbms->motor[num].cycle_cnt++;
		}
		if (rbms->motor[num].delta_pos > 4000) {
			rbms->motor[num].cycle_cnt--;
		}
	}

	rbms->motor[num].raw_rotor_pos = rbms->motor[num].cycle_cnt * 8192
			+ (int16_t) rbms->motor[num].raw_pos - rbms->motor[num].offset_raw_pos;
	rbms->motor[num].rotor_pos = (float) rbms->motor[num].raw_rotor_pos / 8192.0;
	rbms->motor[num].shaft_pos = rbms->motor[num].rotor_pos / rbms->motor[num].type_gear_ratio;
	rbms->motor[num].pos = rbms->motor[num].shaft_pos / rbms->motor[num].config.gear_ratio - rbms->motor[num].offset_pos;
	rbms->motor[num].rpm = (float) rbms->motor[num].raw_rpm
			/ rbms->motor[num].type_gear_ratio;
	rbms->motor[num].current = (float) rbms->motor[num].raw_current
			/ (-rbms->motor[num].input_gradient);
}

/*
 * Function Name		: 	RBMS_CAN_Handler
 * Function Description : 	Called to handle the received CAN data from Robomaster speed controller.
 * Function Remarks		: 	Robomaster Receive Standard ID ranging from 0x201 to 0x208
 * 							Call the function in CANRxFifoPendingCallback function for each RBMS_t *rbms used
 * Function Arguments	: 	ID			,		standard CAN ID
 * 							rxdata		,		CAN receive data
 * 							*rbms		,		pointer to structure RBMS_t
 * Function Return		: 	None
 * Function Example		: 	RBMS_CAN_Handler(CAN1RxMessage.StdId, aData, &rbms1);
 */
void RBMS_CAN_Handler(FDCAN_RxHeaderTypeDef *pRxMsg, uint8_t rxdata[8], uint8_t can) {
	uint8_t index;
	if (pRxMsg->Identifier >= 0x201 && pRxMsg->Identifier <= 0x208) {
		index = pRxMsg->Identifier - 0x201;
//		sprintf(data,"%d\n",index);
//		if (index == 5){
//			led5 = 1;
//		}
//		if (index == 6){
//			led6 = 1;
//		}
//		if (index == 7){
//			led7 = 1;
//		}
//		if (index < 4) {
//				if(can == 1){
//				rbms1.pRxMsg = pRxMsg;
//				memcpy(&rbms1.rxdata, rxdata, pRxMsg->DataLength);
//				RBMS_ACST_Calculation(&rbms1, index);
//				}
//				if(can == 2){
//				rbms2.pRxMsg = pRxMsg;
//				memcpy(&rbms2.rxdata, rxdata, pRxMsg->DataLength);
//				RBMS_ACST_Calculation(&rbms2, index);
//				}
//				if(can == 3){
//				rbms3.pRxMsg = pRxMsg;
//				memcpy(&rbms3.rxdata, rxdata, pRxMsg->DataLength);
//				RBMS_ACST_Calculation(&rbms3, index);
//				}
//			} else {
//				index -= 4;
//				rbms2.pRxMsg = pRxMsg;
//				memcpy(&rbms2.rxdata, rxdata, pRxMsg->DataLength);
//				RBMS_ACST_Calculation(&rbms2, index);
//			}
		if (index < 4) {
			if(can == 1){
			rbms1.pRxMsg = pRxMsg;
			memcpy(&rbms1.rxdata, rxdata, pRxMsg->DataLength);
			RBMS_ACST_Calculation(&rbms1, index);
			}
			if(can == 2){
			rbms3.pRxMsg = pRxMsg;
			memcpy(&rbms3.rxdata, rxdata, pRxMsg->DataLength);
			RBMS_ACST_Calculation(&rbms3, index);
			}
			if(can == 3){
			rbms5.pRxMsg = pRxMsg;
			memcpy(&rbms5.rxdata, rxdata, pRxMsg->DataLength);
			RBMS_ACST_Calculation(&rbms5, index);
			}
		} else {
			index -= 4;
			if(can == 1){
				rbms2.pRxMsg = pRxMsg;
				memcpy(&rbms2.rxdata, rxdata, pRxMsg->DataLength);
				RBMS_ACST_Calculation(&rbms2, index);
			}
			if(can == 2){
				rbms4.pRxMsg = pRxMsg;
				memcpy(&rbms4.rxdata, rxdata, pRxMsg->DataLength);
				RBMS_ACST_Calculation(&rbms4, index);
			}
			if(can == 3){
				rbms6.pRxMsg = pRxMsg;
				memcpy(&rbms6.rxdata, rxdata, pRxMsg->DataLength);
				RBMS_ACST_Calculation(&rbms6, index);
			}
		}


//		led5 = 0;
//		led6 = 0;
//		led7 = 0;
	}
}

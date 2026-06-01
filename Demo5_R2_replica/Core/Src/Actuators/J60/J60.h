/*
 * J60.h
 *
 *  Created on: Apr 18, 2026
 *      Author: Ibrahim Meselhy
 */

#ifndef SRC_ACTUATORS_J60_J60_H_
#define SRC_ACTUATORS_J60_J60_H_

#include <stdint.h>
#include "../../Platform/H7_system.h"
#include "../../BIOS/COM/FDCAN.h"
#include "can_protocol.h"

// --- Full Command Set ---
typedef enum {
    J60_CMD_DISABLE           	  = 1,
    J60_CMD_ENABLE           	  = 2,
    J60_CMD_CALIB_START      	  = 3,
    J60_CMD_CONTROL               = 4,
    J60_CMD_RESET                 = 5,
    J60_CMD_SET_HOME              = 6,
    J60_CMD_SET_GEAR              = 7,
    J60_CMD_SET_ID                = 8,
    J60_CMD_SET_TIMEOUT           = 9,
    J60_CMD_SET_BANDWIDTH         = 10,
    J60_CMD_LIMIT_CURRENT         = 11,
	J60_CMD_UNDER_VOLTAGE		  = 12,
	J60_CMD_OVER_VOLTAGE		  = 13,
	J60_CMD_SET_MOTOR_TEMPERATURE = 14,
    J60_CMD_SAVE_CONFIG      	  = 16,
	J60_CMD_ERROR_RESET		 	  = 17,
    J60_CMD_GET_FW_VERSION   	  = 22,
    J60_CMD_GET_STATUS       	  = 23
} J60_Command_t;


// --- Motor State Structure ---
typedef struct {
    uint8_t id;
    FDCAN_HandleTypeDef *hfdcan;
    float position;      // rad
    float velocity;      // rad/s
    float torque;        // Nm
    float temperature;   // Celsius
    uint16_t error_code;
    uint8_t is_enabled;
    uint32_t last_update_tick;
} J60_Motor_t;


extern J60_Motor_t J60_1;
/*Initialise the motor struct*/
void J60_Init(J60_Motor_t *motor, H7_FDCANHandler_s *h7fdhcan, uint8_t id);

//HAL_StatusTypeDef J60_SendCommand(J60_Motor_t *motor, J60_Command_t cmd, uint8_t *data, uint8_t dlc);

/* Send motion command to the motor:
 * @param: pos    - desired position
 * @param: vel    - desired velocity
 * @param: kp     - desired position gain
 * @param: kd     - desired derivative gain
 * @param: torque - desired torque feedforward
 * */
HAL_StatusTypeDef J60_SetMotion(J60_Motor_t *motor, float pos, float vel, float kp, float kd, float torque);

/* Parse the data received from the motor, this should be called inside the CAN interrupt*/
//Example of code to put inside the CAN interrupt
/*
* uint8_t j60_cmd = (CAN2RxMessage.StdId >> 5);
*   uint8_t j60_id 	= (CAN2RxMessage.StdId & 0x1F);
*
*
* if (j60_id == J60_1.id) {
*
*	// Direct the data based on the command type
*	if (j60_cmd == J60_CMD_CONTROL) {
*		// This updates position, velocity, torque, and temp
*		J60_ParseFeedback(&J60_1, aData, CAN2RxMessage.StdId);
*	}
*	else if (j60_cmd == J60_CMD_GET_STATUS) {
*		// Specifically handle the status/error word
*		J60_1.error_code = (aData[0] | (aData[1] << 8));
*		// You can also extract voltage/current here if needed
*	}
}*/

void J60_ParseFeedback(J60_Motor_t *motor, uint8_t *rxData, uint32_t stdId);

/* Power on the motor and enable the driver stage */
HAL_StatusTypeDef J60_Enable(J60_Motor_t* motor);

/* Power off the motor (coast mode) */
HAL_StatusTypeDef J60_Disable(J60_Motor_t* motor);

/* Set current mechanical position as the 0.0 rad home point */
HAL_StatusTypeDef J60_SetHome(J60_Motor_t* motor);

/* Start the calibration of the motor */
HAL_StatusTypeDef J60_Calibrate(J60_Motor_t* motor);

/* Perform a soft reset of the motor controller */
HAL_StatusTypeDef J60_Reset(J60_Motor_t* motor);

/* Clear current error flags to allow re-enabling */
HAL_StatusTypeDef J60_ClearErrors(J60_Motor_t* motor);

/* Save current settings (ID, Limits, Gear) to NVM */
HAL_StatusTypeDef J60_SaveToFlash(J60_Motor_t* motor);

/* Change the motor's CAN ID (requires SaveToFlash to persist) */
HAL_StatusTypeDef J60_SetID(J60_Motor_t* motor, uint8_t new_id);

/* Set the internal gear ratio (0.0 to 50.0) */
HAL_StatusTypeDef J60_SetGearRatio(J60_Motor_t* motor, float ratio);

/* Set the control loop bandwidth (0 to 1000 Hz) */
HAL_StatusTypeDef J60_SetBandwidth(J60_Motor_t* motor, float hz);

/* Set the CAN watchdog timeout in ms */
HAL_StatusTypeDef J60_SetTimeout(J60_Motor_t* motor, uint8_t ms);

/* Set max current allowed (0.0 to 40.0 Amps) */
HAL_StatusTypeDef J60_SetCurrentLimit(J60_Motor_t* motor, float amps);

/* Set the hardware over-voltage threshold (0.0 to 60.0 Volts) */
HAL_StatusTypeDef J60_SetOverVoltage(J60_Motor_t* motor, float volts);

/* Set the hardware under-voltage threshold (0.0 to 60.0 Volts) */
HAL_StatusTypeDef J60_SetUnderVoltage(J60_Motor_t* motor, float volts);

/* Set the coil temperature shutdown threshold (-20 to 200 C) */
HAL_StatusTypeDef J60_SetMaxMotorTemp(J60_Motor_t* motor, float temp_c);


#endif /* SRC_ACTUATORS_J60_J60_H_ */

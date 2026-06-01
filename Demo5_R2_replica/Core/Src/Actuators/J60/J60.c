/*
 * J60.c
 *
 *  Created on: Apr 18, 2026
 *      Author: Ibrahim Meselhy
 */

#define J60_MAKE_CAN_ID(cmd, id) (((uint32_t)(cmd) << 5) | (uint32_t)(id))




#include "J60.h"

J60_Motor_t J60_1;
// --- Scaling Utilities (Internal) ---
static uint32_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (uint32_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}
/* Used to send CAN packets to the motor */
HAL_StatusTypeDef J60_SendCommand(J60_Motor_t *motor, J60_Command_t cmd, uint8_t *data, uint8_t len) {
	H7_state_e rslt;
	rslt = FDCAN_TxMsg(motor->hfdcan, (cmd << 5) | motor->id, data, len);
	if(rslt != H7_OK){
		return (rslt == H7_FDCAN_TIMEOUT_ERR)? HAL_TIMEOUT : HAL_ERROR;
	}
	return HAL_OK;

}

/**
 * Sends a 16-bit configuration value to the motor.
 * Used for Gear, Current, Voltage, and Temp limits.
 */
HAL_StatusTypeDef J60_SetConfig16(J60_Motor_t* motor, J60_Command_t cmd,
                                 float value, float min, float max) {
    uint8_t data[2];

    // Scale the float to a 16-bit integer
    uint16_t scaled_val = float_to_uint(value, min, max, 16);

    // Pack as Little-Endian
    data[0] = (uint8_t)(scaled_val & 0xFF);
    data[1] = (uint8_t)(scaled_val >> 8);

    return J60_SendCommand(motor, cmd, data, 2);
}
// --- Initialization ---
void J60_Init(J60_Motor_t *motor, H7_FDCANHandler_s *h7fdhcan, uint8_t id) {
    motor->id = id;
    motor->hfdcan = h7fdhcan->hfdcan;
    motor->is_enabled = 1;
}



// --- Motion Command Packing (CMD 4) ---
HAL_StatusTypeDef J60_SetMotion(J60_Motor_t *motor, float pos, float vel, float kp, float kd, float torque) {
    uint8_t data[8];

    uint16_t p = float_to_uint(pos, -40.0f, 40.0f, 16);
    uint16_t v = float_to_uint(vel, -40.0f, 40.0f, 14);
    uint16_t kp_uint = float_to_uint(kp, 0.0f, 1023.0f, 10);
    uint16_t kd_uint = float_to_uint(kd, 0.0f, 51.0f, 8);
    uint16_t t = float_to_uint(torque, -40.0f, 40.0f, 16);

    data[0] = p & 0xFF;
    data[1] = p >> 8;
    data[2] = v & 0xFF;
    data[3] = ((v >> 8) & 0x3F) | ((kp_uint & 0x03) << 6);
    data[4] = kp_uint >> 2;
    data[5] = kd_uint;
    data[6] = t & 0xFF;
    data[7] = t >> 8;

    return J60_SendCommand(motor, J60_CMD_CONTROL, data, 8);
}

// --- Feedback Unpacking ---
void J60_ParseFeedback(J60_Motor_t *motor, uint8_t *rxData, uint32_t stdId) {
    uint8_t cmd = stdId >> 5;

    if (cmd == J60_CMD_CONTROL) {
        uint32_t p_raw = rxData[0] | (rxData[1] << 8) | ((rxData[2] & 0x0F) << 16);
        uint32_t v_raw = ((rxData[2] & 0xF0) >> 4) | (rxData[3] << 4) | (rxData[4] << 12);
        uint16_t t_raw = rxData[5] | (rxData[6] << 8);

        motor->position = uint_to_float(p_raw, -40.0f, 40.0f, 20);
        motor->velocity = uint_to_float(v_raw, -40.0f, 40.0f, 20);
        motor->torque   = uint_to_float(t_raw, -40.0f, 40.0f, 16);
        motor->temperature = uint_to_float((rxData[7] >> 1), -20.0f, 200.0f, 7);
        motor->last_update_tick = HAL_GetTick();
    }
}

HAL_StatusTypeDef J60_Enable(J60_Motor_t* motor) {
    return J60_SendCommand(motor, J60_CMD_ENABLE, NULL, 0);
}

HAL_StatusTypeDef J60_Disable(J60_Motor_t* motor) {
    return J60_SendCommand(motor, J60_CMD_DISABLE, NULL, 0);
}

HAL_StatusTypeDef J60_SetHome(J60_Motor_t* motor) {
    return J60_SendCommand(motor, J60_CMD_SET_HOME, NULL, 0);
}

HAL_StatusTypeDef J60_Calibrate(J60_Motor_t* motor) {
	return J60_SendCommand(motor, J60_CMD_CALIB_START, NULL, 0);
}

HAL_StatusTypeDef J60_Reset(J60_Motor_t* motor) {
    return J60_SendCommand(motor, J60_CMD_RESET, NULL, 0);
}

HAL_StatusTypeDef J60_ClearErrors(J60_Motor_t* motor) {
    return J60_SendCommand(motor, J60_CMD_ERROR_RESET, NULL, 0);
}

HAL_StatusTypeDef J60_SaveToFlash(J60_Motor_t* motor) {
    return J60_SendCommand(motor, J60_CMD_SAVE_CONFIG, NULL, 0);
}

HAL_StatusTypeDef J60_SetID(J60_Motor_t* motor, uint8_t new_id) {
    uint8_t data = new_id;
    HAL_StatusTypeDef st;
    st = J60_SendCommand(motor, J60_CMD_SET_ID, &data, 1);
    if(st == HAL_OK) motor->id = new_id;
    return st;
}

HAL_StatusTypeDef J60_SetGearRatio(J60_Motor_t* motor, float ratio) {
    return J60_SetConfig16(motor, J60_CMD_SET_GEAR, ratio, 0.0f, 50.0f);
}

HAL_StatusTypeDef J60_SetBandwidth(J60_Motor_t* motor, float hz) {
    return J60_SetConfig16(motor, J60_CMD_SET_BANDWIDTH, hz, 0.0f, 1000.0f);
}

HAL_StatusTypeDef J60_SetTimeout(J60_Motor_t* motor, uint8_t ms) {
    // 1 unit usually equals 10ms in these drivers
    uint8_t data = ms;
    return J60_SendCommand(motor, J60_CMD_SET_TIMEOUT, &data, 1);
}

HAL_StatusTypeDef J60_SetCurrentLimit(J60_Motor_t* motor, float amps) {
    return J60_SetConfig16(motor, J60_CMD_LIMIT_CURRENT, amps, 0.0f, 40.0f);
}

HAL_StatusTypeDef J60_SetOverVoltage(J60_Motor_t* motor, float volts) {
    return J60_SetConfig16(motor, J60_CMD_OVER_VOLTAGE, volts, 0.0f, 60.0f);
}

HAL_StatusTypeDef J60_SetUnderVoltage(J60_Motor_t* motor, float volts) {
    return J60_SetConfig16(motor, J60_CMD_UNDER_VOLTAGE, volts, 0.0f, 60.0f);
}

HAL_StatusTypeDef J60_SetMaxMotorTemp(J60_Motor_t* motor, float temp_c) {
    return J60_SetConfig16(motor, J60_CMD_SET_MOTOR_TEMPERATURE, temp_c, -20.0f, 200.0f);
}

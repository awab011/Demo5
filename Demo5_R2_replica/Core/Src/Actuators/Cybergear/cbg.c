/*
 * cbg.c
 *
 *  Created on: Jul 29, 2024
 *      Author: moh
 *      Modified: Jin Ye Leong
 *
 *  Modified for H7 compatibility by Mohammed Abdulalem on Dec 17, 2025
 */

/*
 *
 * loc_kp = 30
 * spd_kp = 1
 * spd_ki = 0.002
 *
 * To run Cybergear:
 * Cybergear_enable(&cybergear, 0);
 * Cybergear_set_run_mode(&cybergear, 0, MODE_MOTION);
 * Cybergear_send_motion_command(&cybergear, 0, 0, 110, 0, 0, 1);
 *
 */

#include "../../Actuators/Cybergear/cbg.h"

#include "../../adapter.h"


// Global

Cybergear cybergear1, cybergear2;


static int float_to_uint(float x, float min, float max, int bits)
{
    float span = max - min;
    x = (x < min) ? min : ((x > max) ? max : x);
    return (int)((x - min) * ((1 << bits) - 1) / span);
}

static float uint_to_float(uint16_t x, float min, float max)
{
    return min + (float)x * (max - min) / 0xFFFF;
}

uint8_t Cybergear_init(Cybergear* cybergear, FDCAN_HandleTypeDef *hfdcan, uint8_t masterID, uint8_t motorID) {
//    if (numMotors > MAX_MOTORS) {
//        return false; // Maximum of MAX_MOTORS motors supported
//    }

    cybergear->masterID = masterID;
//    cybergear->numMotors = numMotors;

	cybergear->driver.hfdcan = hfdcan;
	cybergear->driver.master_can_id = masterID;
	cybergear->driver.target_can_id = motorID;
	memset(&cybergear->driver.txdata,0,8);
	memset(&cybergear->driver.motor_status,0,sizeof(MotorStatus));
	memset(&cybergear->driver.motor_param,0,sizeof(MotorParam));
	memset(&cybergear->driver.motor_fault,0,sizeof(MotorFault));

    return true;
}


uint8_t Cybergear_enable(Cybergear* cybergear) {
    // Enable the master and all associated motors

    cybergear->driver.enable = 1;
	memset(&cybergear->driver.txdata, 0, 8);
	CybergearDriver_send_command(&cybergear->driver, CMD_ENABLE, cybergear->driver.master_can_id);

    return true;
}


void CybergearDriver_send_command(CybergearDriver *driver, uint8_t cmd_id, uint16_t option) {
    uint32_t id = (cmd_id << 24) | (option << 8) | driver->target_can_id;
    driver->motor_status.motor_update_flag = 0;
    driver->pTxMsg.Identifier = id;
    FDCAN_TxMsgEID(driver->hfdcan, id, driver->txdata, 8);
    driver->send_count++;
    HAL_Delay(1);
//    osDelay(1);
    uint32_t time_sent = HAL_GetTick();
    while(!driver->motor_status.motor_update_flag && (HAL_GetTick()-time_sent <= 3)){
    }
}


uint8_t Cybergear_set_run_mode(Cybergear* cybergear, uint8_t mode) {
//	if (motorIndex < 0 || motorIndex >= cybergear->numMotors) {
//		return false; // Invalid motor index
//	}
	cybergear->driver.run_mode = mode;
	uint8_t data[8] = { 0x00 };
	data[0] = ADDR_RUN_MODE & 0x00FF;
	data[1] = ADDR_RUN_MODE >> 8;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = cybergear->driver.run_mode;
	memcpy(&cybergear->driver.txdata, data, 8);
	CybergearDriver_send_command(&cybergear->driver, CMD_RAM_WRITE, cybergear->driver.master_can_id);

    return true;
}


uint8_t Cybergear_send_motion_command(Cybergear* cybergear, float pos, float vel, float torque, float kp, float kd) {
//    if (motorIndex < 0 || motorIndex >= cybergear->numMotors) {
//        return false; // Invalid motor index
//    }

//	CybergearDriver_set_control(&cybergear->drivers[motorIndex], pos, vel, torque, kp, kd);
	if(cybergear->driver.enable){
		pos -= cybergear->driver.pos_offset; //for future use
		pos *= 2*PI;
		vel *= 2*PI/60.0;

		uint8_t data[8] = {0x00};
		data[0] = float_to_uint(pos, P_MIN, P_MAX, 16) >> 8;
		data[1] = float_to_uint(pos, P_MIN, P_MAX, 16);
		data[2] = float_to_uint(vel, V_MIN, V_MAX, 16) >> 8;
		data[3] = float_to_uint(vel, V_MIN, V_MAX, 16);
		data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
		data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
		data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
		data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);
		memcpy(&cybergear->driver.txdata, data, 8);
		uint16_t data_torque = float_to_uint(torque, T_MIN, T_MAX, 16);
		CybergearDriver_send_command(&cybergear->driver, CMD_CONTROL, data_torque);
	}
	return true;
}


uint8_t Cybergear_disable(Cybergear* cybergear) {
	//disable motor:
	cybergear->driver.enable = 0;
	memset(&cybergear->driver.txdata, 0, 8);
	CybergearDriver_send_command(&cybergear->driver, CMD_DISABLE, cybergear->driver.master_can_id);

	//clear error:
//	memset(&cybergear->drivers[motorIndex].txdata, 0, 8);
	cybergear->driver.txdata[0] = 1;
	CybergearDriver_send_command(&cybergear->driver, CMD_DISABLE, cybergear->driver.master_can_id);

    return true;
}


void Cybergear_FDCAN_Handler(Cybergear* cybergear, FDCAN_RxHeaderTypeDef *pRxMsg, uint8_t rxdata[]) {
	uint32_t id = pRxMsg->Identifier;
	uint8_t motor_can_id = (id & 0xFF00) >> 8;
	uint8_t cbgerr = (id & 0xFF0000) >> 16;
	uint8_t comtype = (id & 0xFF000000) >> 24;
//	for (uint8_t i = 0; i < cybergear.numMotors; i++) {
	if (motor_can_id == cybergear->driver.target_can_id) {
		cybergear->driver.com_type = comtype;
		memcpy(&cybergear->driver.rxdata, rxdata, 8);

		if (cybergear->driver.com_type == 2) {
			cybergear->driver.motor_status.motor_update_flag = 1;
			cybergear->driver.motor_status.motor_id = motor_can_id;
			cybergear->driver.motor_status.motor_mode = (cbgerr & 0b11000000) >> 6;
			cybergear->driver.motor_fault.encoder_not_calibrated = (cbgerr & 0b00100000) >> 5;
			cybergear->driver.motor_fault.hall_encoder_fault = (cbgerr & 0b00010000) >> 4;
			cybergear->driver.motor_fault.magnetic_encoder_fault = (cbgerr & 0b00001000) >> 3;
			cybergear->driver.motor_fault.motor_over_tempareture = (cbgerr & 0b00000100) >> 2;
			cybergear->driver.motor_fault.over_current = (cbgerr & 0b00000010) >> 1;
			cybergear->driver.motor_fault.under_voltage = (cbgerr & 0b00000001);
			cybergear->driver.motor_status.raw_position = (cybergear->driver.rxdata[0]<<8) + cybergear->driver.rxdata[1];
			cybergear->driver.motor_status.raw_velocity = (cybergear->driver.rxdata[2]<<8) + cybergear->driver.rxdata[3];
			cybergear->driver.motor_status.raw_torque = (cybergear->driver.rxdata[4]<<8) + cybergear->driver.rxdata[5];
			cybergear->driver.motor_status.raw_temperature = (cybergear->driver.rxdata[6]<<8) + cybergear->driver.rxdata[7];
			cybergear->driver.motor_status.motor_update_flag = 0;

			cybergear->driver.motor_status.position = ((uint_to_float(cybergear->driver.motor_status.raw_position, P_MIN, P_MAX)) * 180.0/3.1415926) / 360.0;
			cybergear->driver.motor_status.velocity = uint_to_float(cybergear->driver.motor_status.raw_velocity,  V_MIN, V_MAX) * 3.1415926 / 30;
			cybergear->driver.motor_status.torque = uint_to_float(cybergear->driver.motor_status.raw_torque, T_MIN, T_MAX);
			cybergear->driver.motor_status.temperature = cybergear->driver.motor_status.raw_temperature;
		}

		else if(cybergear->driver.com_type == 17){
			cybergear->driver.index = (cybergear->driver.rxdata[1] << 8) + cybergear->driver.rxdata[0];
			switch (cybergear->driver.index) {

			case ADDR_RUN_MODE:
				memcpy(&cybergear->driver.motor_param.run_mode, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_IQ_REF:
				memcpy(&cybergear->driver.motor_param.mech_pos, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_SPEED_REF:
				memcpy(&cybergear->driver.motor_param.spd_ref, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_MECH_POS:
				memcpy(&cybergear->driver.motor_param.mech_pos, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_LIMIT_TORQUE:
				memcpy(&cybergear->driver.motor_param.limit_torque, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_CURRENT_KP:
				memcpy(&cybergear->driver.motor_param.cur_kp, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_CURRENT_KI:
				memcpy(&cybergear->driver.motor_param.cur_ki, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_CURRENT_FILTER_GAIN:
				memcpy(&cybergear->driver.motor_param.cur_filt_gain, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_LOC_REF:
				memcpy(&cybergear->driver.motor_param.loc_ref, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_LIMIT_SPEED:
				memcpy(&cybergear->driver.motor_param.limit_spd, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_LIMIT_CURRENT:
				memcpy(&cybergear->driver.motor_param.limit_cur, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_IQF:
				memcpy(&cybergear->driver.motor_param.iqf, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_MECH_VEL:
				memcpy(&cybergear->driver.motor_param.mech_vel, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_VBUS:
				memcpy(&cybergear->driver.motor_param.vbus, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_ROTATION:
				memcpy(&cybergear->driver.motor_param.rotation, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_LOC_KP:
				memcpy(&cybergear->driver.motor_param.loc_kp, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_SPD_KP:
				memcpy(&cybergear->driver.motor_param.spd_kp, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			case ADDR_SPD_KI:
				memcpy(&cybergear->driver.motor_param.spd_ki, &cybergear->driver.rxdata[4], sizeof(float));
				break;
			}
		}
	}
//	}
}

void Cybergear_send_speed_command(Cybergear *cybergear, float current_limit, float spd_kp, float spd_ki, float spd_ref){
	spd_ref *= 2*PI/60.0;
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_SPD_KP, spd_kp);
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_SPD_KI, spd_ki);
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_LIMIT_CURRENT, current_limit);
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_SPEED_REF, spd_ref);
}

void Cybergear_send_pos_command(Cybergear *cybergear, float speed_limit, float loc_kp, float spd_kp, float spd_ki, float loc_ref){
	loc_ref -= cybergear->driver.pos_offset; //for future use
	loc_ref *= 2*PI;
	speed_limit *= 2*PI/60.0;
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_LIMIT_SPEED, speed_limit);
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_LOC_KP, loc_kp);
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_SPD_KP, spd_kp);
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_SPD_KI, spd_ki);
	CybergearDriver_write_param_float(&cybergear->driver, ADDR_LOC_REF, loc_ref);
}

void CybergearDriver_write_param_float(CybergearDriver *driver, uint16_t addr, float value)
{
	memset(&driver->txdata, 0, 8);
	driver->txdata[0] = addr & 0xFF;
	driver->txdata[1] = (addr >> 8) & 0xFF;

	switch (addr){
	case ADDR_IQ_REF:
		Ulim(value, IQ_REF_MAX);
		Llim(value, -IQ_REF_MIN);
		break;
	case ADDR_SPEED_REF:
		Ulim(value, SPD_REF_MAX);
		Llim(value, -SPD_REF_MIN);
		break;
	case ADDR_LIMIT_TORQUE:
		Ulim(value, LIMIT_TORQUE_MAX);
		Llim(value, -LIMIT_TORQUE_MIN);
		break;
	case ADDR_CURRENT_KP:
		Ulim(value, CUR_KP_MAX);
		Llim(value, -CUR_KP_MIN);
		break;
	case ADDR_CURRENT_KI:
		Ulim(value, CUR_KI_MAX);
		Llim(value, -CUR_KI_MIN);
		break;
	case ADDR_CURRENT_FILTER_GAIN:
		Ulim(value, CURRENT_FILTER_GAIN_MAX);
		Llim(value, -CURRENT_FILTER_GAIN_MIN);
		break;
	case ADDR_LIMIT_SPEED:
		Ulim(value, LIMIT_SPD_MAX);
		Llim(value, -LIMIT_SPD_MIN);
		break;
	case ADDR_LIMIT_CURRENT:
		Ulim(value, LIMIT_CUR_MAX);
		Llim(value, -LIMIT_CUR_MIN);
		break;
	default:
		break;
	}

    memcpy(&driver->txdata[4], &value, sizeof(float));
    CybergearDriver_send_command(driver, CMD_RAM_WRITE, driver->master_can_id);
}


uint8_t Cybergear_set_mech_pos_zero(Cybergear* cybergear){
//	if (motorIndex < 0 || motorIndex >= cybergear->numMotors) {
//		return false; // Invalid motor index
//	}
	memset(&cybergear->driver.txdata, 0, 8);
	cybergear->driver.txdata[0] = 1;
	CybergearDriver_send_command(&cybergear->driver, CMD_SET_MECH_POSITION_TO_ZERO, cybergear->driver.master_can_id);
    return true;
}

void Cybergear_read_all_param(Cybergear* cybergear){

		CybergearDriver_read_param(&cybergear->driver, ADDR_RUN_MODE);
		CybergearDriver_read_param(&cybergear->driver, ADDR_IQ_REF);
		CybergearDriver_read_param(&cybergear->driver, ADDR_SPEED_REF);
		CybergearDriver_read_param(&cybergear->driver, ADDR_LIMIT_TORQUE);
		CybergearDriver_read_param(&cybergear->driver, ADDR_CURRENT_KP);
		CybergearDriver_read_param(&cybergear->driver, ADDR_CURRENT_KI);
		CybergearDriver_read_param(&cybergear->driver, ADDR_CURRENT_FILTER_GAIN);
		CybergearDriver_read_param(&cybergear->driver, ADDR_LOC_REF);
		CybergearDriver_read_param(&cybergear->driver, ADDR_LIMIT_SPEED);
		CybergearDriver_read_param(&cybergear->driver, ADDR_LIMIT_CURRENT);
		CybergearDriver_read_param(&cybergear->driver, ADDR_MECH_POS);
		CybergearDriver_read_param(&cybergear->driver, ADDR_IQF);
		CybergearDriver_read_param(&cybergear->driver, ADDR_MECH_VEL);
		CybergearDriver_read_param(&cybergear->driver, ADDR_VBUS);
		CybergearDriver_read_param(&cybergear->driver, ADDR_ROTATION);
		CybergearDriver_read_param(&cybergear->driver, ADDR_LOC_KP);
		CybergearDriver_read_param(&cybergear->driver, ADDR_SPD_KP);
		CybergearDriver_read_param(&cybergear->driver, ADDR_SPD_KI);

}

void CybergearDriver_read_param(CybergearDriver *driver, uint16_t addr) {

	memset(&driver->txdata, 0, 8);
	driver->txdata[0] = addr & 0x00FF;
	driver->txdata[1] = addr >> 8;

	CybergearDriver_send_command(driver, CMD_RAM_READ, driver->master_can_id);
}


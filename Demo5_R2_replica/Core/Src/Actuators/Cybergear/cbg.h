/*
 * cbg.h
 *
 *  Created on: Jul 29, 2024
 *      Author: moh
 *      Modified: Jin Ye Leong
 *
 *  Modified for H7 compatibility by Mohammed Abdulalem on Dec 17, 2025
 */

#ifndef SRC_CYBERGEAR_CBG_H_
#define SRC_CYBERGEAR_CBG_H_

//#include "../BIOS/system.h"
#include "../../Actuators/Cybergear/cybergear_defs.h"
#include "../../BIOS/COM/FDCAN.h"

/* These macros are already in H7_system.h, so I commented them (Mohammed) */
//#define Llim(a, b)	a = ((a <= -b)? -b : a);
//#define Ulim(a, b)	a = ((a >= b) ?  b : a);

typedef struct {
    uint8_t encoder_not_calibrated;
    uint8_t over_current;
    uint8_t under_voltage;
    uint8_t driver_chip;
    uint8_t motor_over_tempareture;
    uint8_t hall_encoder_fault;
    uint8_t magnetic_encoder_fault;
} MotorFault;

typedef struct {
    uint16_t raw_position;
    uint16_t raw_velocity;
    uint16_t raw_torque;
    uint16_t raw_temperature;
    float position;
    float velocity;
    float torque;
    float temperature;
    uint8_t motor_id;
    uint8_t motor_mode;
    uint32_t stamp_usec;
    uint8_t motor_update_flag;
} MotorStatus;

typedef struct {
    uint8_t run_mode;
    float iq_ref;
    float spd_ref;
    float limit_torque;
    float cur_kp;
    float cur_ki;
    float cur_filt_gain;
    float loc_ref;
    float limit_spd;
    float limit_cur;
    float mech_pos;
    float iqf;
    float mech_vel;
    float vbus;
    int16_t rotation;
    float loc_kp;
    float spd_kp;
    float spd_ki;
    uint32_t stamp_usec;
} MotorParam;

typedef struct {
	FDCAN_HandleTypeDef *hfdcan;
    uint8_t master_can_id;
    uint8_t target_can_id;
    uint8_t run_mode;
    uint32_t send_count;
    MotorStatus motor_status;
    MotorParam motor_param;
    MotorFault motor_fault;
    FDCAN_RxHeaderTypeDef pRxMsg;
	FDCAN_TxHeaderTypeDef pTxMsg;
    uint8_t rxdata[8];
    uint8_t txdata[8];
    uint8_t disconnected;
    uint8_t enable;
    uint8_t com_type;
    float pos_offset;
    uint16_t index;
} CybergearDriver;

#define MAX_MOTORS 8

typedef struct {
	uint8_t masterID;
//    CybergearDriver drivers[MAX_MOTORS];
//    uint8_t numMotors;
	CybergearDriver driver;
    char UartMsg[100];
} Cybergear;

// Global
extern Cybergear cybergear1, cybergear2;

uint8_t Cybergear_init(Cybergear* cybergear, FDCAN_HandleTypeDef *hfdcan, uint8_t masterID, uint8_t motorID);
uint8_t Cybergear_enable(Cybergear* cybergear);
void CybergearDriver_send_command(CybergearDriver *driver, uint8_t cmd_id, uint16_t option);
uint8_t Cybergear_set_run_mode(Cybergear* cybergear, uint8_t mode);
uint8_t Cybergear_send_motion_command(Cybergear* cybergear, float pos, float vel, float torque, float kp, float kd);
uint8_t Cybergear_disable(Cybergear* cybergear);
void Cybergear_FDCAN_Handler(Cybergear* cybergear, FDCAN_RxHeaderTypeDef *pRxMsg, uint8_t rxdata[]);
void Cybergear_send_speed_command(Cybergear *cybergear, float current_limit, float spd_kp, float spd_ki, float spd_ref);
void Cybergear_send_pos_command(Cybergear *cybergear, float speed_limit, float loc_kp, float spd_kp, float spd_ki, float loc_ref);
void CybergearDriver_write_param_float(CybergearDriver *driver, uint16_t addr, float value);
uint8_t Cybergear_set_mech_pos_zero(Cybergear* cybergear);
void Cybergear_read_all_param(Cybergear* cybergear);
void CybergearDriver_read_param(CybergearDriver *driver, uint16_t addr);

#endif /* SRC_CYBERGEAR_CBG_H_ */

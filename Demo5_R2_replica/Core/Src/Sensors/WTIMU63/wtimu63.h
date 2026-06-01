/*
 * wtimu63.h
 *
 *  Created on: Mar 9, 2026
 *      Author: Szeyi
 */

#ifndef SRC_IMU_WTIMU63_H_
#define SRC_IMU_WTIMU63_H_

#include "../../BIOS/BIOS.h"

//#include "../KF/KF.h"
//#include "../Moving_Average/mov_ave.h"

#define ACC            0x51
#define GYRO           0x52
#define ANGLE          0x53
#define MAG            0x54


typedef enum {
    WTIMU63_HEADER_PENDING = 0,
    WTIMU63_TYPE_PENDING,
    WTIMU63_DATA_PENDING
} WTIMU63_State_t;

typedef struct {

    UART_HandleTypeDef*huartx; //header+type+8 data+checksum
    I2C_HandleTypeDef*hi2cimu;

    uint8_t Buffer[18];
    uint8_t rx_byte;
    uint8_t buf_idx;
    WTIMU63_State_t State;

    volatile uint8_t checksum;

    volatile float roll;
    volatile float pitch;
    volatile float yaw;

    volatile float roll_rate;
    volatile float pitch_rate;
    volatile float yaw_rate;

    volatile float x_acc;
    volatile float y_acc;
    volatile float z_acc;

    volatile float x_mag;
    volatile float y_mag;
    volatile float z_mag;

    volatile float prev_yaw;
    volatile float yaw_constant;
    volatile float offset;
    volatile float real_z;
    volatile float real_zrad;
} WTIMU63_t;

extern WTIMU63_t IMU63;
void WTIMU63_Init(WTIMU63_t *IMU, UART_HandleTypeDef *huartx);
void WTIMU63_InitI2C(WTIMU63_t *IMU, I2C_HandleTypeDef  *hi2c);
void WTIMU63_InitI2C_DMA(WTIMU63_t *IMU, I2C_HandleTypeDef  *hi2c);
void WTIMU63_Handler(WTIMU63_t *IMU);
void WTIMU63_I2CHandle(WTIMU63_t *IMU);
void WTIMU63_DMAHandle(WTIMU63_t *IMU);
void WTIMU63_SetOffset(WTIMU63_t *IMU, float offset);
void WTIMU63_ResetYawTracking(WTIMU63_t *IMU);

#endif /* SRC_IMU_WTIMU63_H_ */

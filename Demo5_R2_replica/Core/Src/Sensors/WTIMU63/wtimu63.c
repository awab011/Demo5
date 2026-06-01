/*
 * wtimu63.c
 *
 *  Created on: Mar 9, 2026
 *      Author: Szeyi
 */
#include "wtimu63.h"

float dbg_yaw   = 0.0f;
float dbg_pitch = 0.0f;
float dbg_roll  = 0.0f;
float dbg_realz = 0.0f;

//float dbg_mag_x = 0.0f;
//float dbg_mag_y = 0.0f;
//float dbg_mag_z = 0.0f;

uint8_t  wimu_i2c_raw[18] = {0};

uint8_t wimu_consec_pass = 0;
uint32_t wimu_cs_pass = 0;
uint32_t wimu_cs_fail = 0;
uint8_t wimu_last_type = 0;
uint8_t wimu_last_angle_buf[11] = {0};
WTIMU63_t IMU63;
static void WTIMU63_UpdateYaw(WTIMU63_t *IMU)
{
    // Only keep range check
    if(IMU->yaw < -180.0f || IMU->yaw > 180.0f) return;

    // Clamp yaw_constant to prevent runaway
    if(IMU->yaw_constant > 20)  IMU->yaw_constant = 20;
    if(IMU->yaw_constant < -20) IMU->yaw_constant = -20;

    if (IMU->yaw < -150.0f) {
        if (IMU->prev_yaw > 150.0f) IMU->yaw_constant++;
    } else if (IMU->yaw > 150.0f) {
        if (IMU->prev_yaw < -150.0f) IMU->yaw_constant--;
    }

    IMU->prev_yaw  = IMU->yaw;
    IMU->real_z    = IMU->yaw + IMU->yaw_constant * 360.0f + IMU->offset;
    IMU->real_zrad = (IMU->real_z / 180.0f) * 3.141593f;

    dbg_yaw   = IMU->yaw;
    dbg_pitch = IMU->pitch;
    dbg_roll  = IMU->roll;
    dbg_realz = IMU->real_z;
}
/*
 * Function Name		: WTIMU63_Init
 * Function Description	: Called to init imu using UART interrupt mode.
 *						  State starts at WTIMU63_HEADER_PENDING.
 * Function Remarks		: Call once in main after MX_UARTx_Init().
 * Function Arguments	: *IMU		, pointer to structure WTIMU63_t
 *						  *huartx	, pointer to HAL UART handle
 * Function Return		: None
 * Function Example		: WTIMU63_Init(&IMU1, &huart2);
 */
void WTIMU63_Init(WTIMU63_t *IMU, UART_HandleTypeDef *huartx)
{
	IMU->huartx = huartx;
	IMU->State = WTIMU63_HEADER_PENDING;
	IMU->checksum = 0;
	IMU->offset = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw = 0;
	IMU->buf_idx = 0;
	HAL_UART_Receive_IT(IMU->huartx, &IMU->rx_byte, 1);
}

/*
 * Function Name		: WTIMU63_InitI2C
 * Function Description	: Called to init WTIMU63 using I2C interrupt mode.
 *						  18 bytes starting from register 0x34.
 * Function Remarks		: Call once in main after MX_I2Cx_Init().
 *						  Default I2C address is 0x50 (7-bit).
 * Function Arguments	: *IMU		, pointer to structure WTIMU63_t
 *						  *hi2c		, pointer to HAL I2C handle
 * Function Return		: None
 * Function Example		: WTIMU63_InitI2C(&IMU1, &hi2c1);
 */
void WTIMU63_InitI2C(WTIMU63_t *IMU, I2C_HandleTypeDef *hi2c)
{
	IMU->hi2cimu = hi2c;
	IMU->checksum = 0;
	IMU->offset = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw = 0;
	HAL_I2C_Mem_Read_IT(IMU->hi2cimu,(0x50 << 1),0x34,I2C_MEMADD_SIZE_8BIT, (uint8_t *)IMU->Buffer, 18);
}

/*
 * Function Name		: WTIMU63_InitI2C_DMA
 * Function Description	: Called to init WTIMU63 using I2C DMA mode.
 *						  Same as WTIMU63_InitI2C but uses DMA transfer
 * Function Remarks		: Call once in main after MX_I2Cx_Init().
 * Function Arguments	: *IMU		, pointer to structure WTIMU63_t
 *						  *hi2c		, pointer to HAL I2C handle
 * Function Return		: None
 * Function Example		: WTIMU63_InitI2C_DMA(&IMU1, &hi2c1);
 */
void WTIMU63_InitI2C_DMA(WTIMU63_t *IMU, I2C_HandleTypeDef *hi2c)
{
	IMU->hi2cimu      = hi2c;
	IMU->checksum     = 0;
	IMU->offset       = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw     = 0;
	HAL_I2C_Mem_Read_DMA(IMU->hi2cimu,(0x50 << 1),0x34,I2C_MEMADD_SIZE_8BIT, (uint8_t *)IMU->Buffer, 18);
}

/*
 * Function Name		: WTIMU63_Handler
 * Function Description	: Called to handle incoming UART bytes from WTIMU63.
 *						    WTIMU63_HEADER_PENDING -> wait for 0x55
 *						    WTIMU63_TYPE_PENDING -> validate type byte (0x51/0x52/0x53)
 *						    WTIMU63_DATA_PENDING -> collect 8 data bytes + checksum,
 *						  decodes into IMU->roll, pitch, yaw, x_acc, y_acc, z_acc, roll_rate, pitch_rate, yaw_rate, real_z, real_zrad.
 * Function Remarks		: Call inside HAL_UART_RxCpltCallback() only when
 *						  huart->Instance matches IMU->huartx->Instance.
 * Function Arguments	: *IMU
 * Function Return		: None
 * Function Example		: void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
 *						      if(huart->Instance == IMU.huartx->Instance)
 *						          WTIMU63_Handler(&IMU);
 *						  }
 */
void WTIMU63_Handler(WTIMU63_t *IMU)
{
    switch(IMU->State)
    {
    case WTIMU63_HEADER_PENDING:
        IMU->buf_idx = 0;  // always reset on header state
        if(IMU->rx_byte == 0x55)
        {
            IMU->Buffer[0] = 0x55;
            IMU->buf_idx   = 1;
            IMU->State     = WTIMU63_TYPE_PENDING;
        }
        HAL_UART_Receive_IT(IMU->huartx, &IMU->rx_byte, 1);
        break;

    case WTIMU63_TYPE_PENDING:
        wimu_last_type = IMU->rx_byte;

        if(IMU->rx_byte >= 0x50 && IMU->rx_byte <= 0x5F)
        {
            IMU->Buffer[1] = IMU->rx_byte;
            IMU->buf_idx   = 2;
            IMU->State     = WTIMU63_DATA_PENDING;
        }
        else if(IMU->rx_byte == 0x55)
        {
            IMU->Buffer[0] = 0x55;
            IMU->buf_idx   = 1;
        }
        else
        {
            IMU->State   = WTIMU63_HEADER_PENDING;
            IMU->buf_idx = 0;
        }
        HAL_UART_Receive_IT(IMU->huartx, &IMU->rx_byte, 1);
        break;

    case WTIMU63_DATA_PENDING:

        if(IMU->buf_idx >= 11)
        {
            IMU->State   = WTIMU63_HEADER_PENDING;
            IMU->buf_idx = 0;
            HAL_UART_Receive_IT(IMU->huartx, &IMU->rx_byte, 1);
            break;
        }

        IMU->Buffer[IMU->buf_idx++] = IMU->rx_byte;

        if(IMU->buf_idx < 11)
        {
            HAL_UART_Receive_IT(IMU->huartx, &IMU->rx_byte, 1);
            break;
        }

        {
            uint16_t sum = 0;
            for(int i = 0; i < 10; i++)
                sum += IMU->Buffer[i];
            IMU->checksum = (uint8_t)(sum & 0xFF);
        }

        if(IMU->checksum == IMU->Buffer[10])
        {
            wimu_cs_pass++;
            wimu_consec_pass++;

            if(wimu_consec_pass >= 3)
            {
                uint8_t type = IMU->Buffer[1];
                if(type == ACC)
                {
                    IMU->x_acc = (int16_t)((IMU->Buffer[3]<<8)|IMU->Buffer[2]) / 32768.0f * 16.0f * 9.8f;
                    IMU->y_acc = (int16_t)((IMU->Buffer[5]<<8)|IMU->Buffer[4]) / 32768.0f * 16.0f * 9.8f;
                    IMU->z_acc = (int16_t)((IMU->Buffer[7]<<8)|IMU->Buffer[6]) / 32768.0f * 16.0f * 9.8f;
                }
                else if(type == GYRO)
                {
                    IMU->roll_rate  = (int16_t)((IMU->Buffer[3]<<8)|IMU->Buffer[2]) / 32768.0f * 2000.0f;
                    IMU->pitch_rate = (int16_t)((IMU->Buffer[5]<<8)|IMU->Buffer[4]) / 32768.0f * 2000.0f;
                    IMU->yaw_rate   = (int16_t)((IMU->Buffer[7]<<8)|IMU->Buffer[6]) / 32768.0f * 2000.0f;
                }
                else if(type == ANGLE)
                {
                    IMU->roll  = (int16_t)((IMU->Buffer[3]<<8)|IMU->Buffer[2]) / 32768.0f * 180.0f;
                    IMU->pitch = (int16_t)((IMU->Buffer[5]<<8)|IMU->Buffer[4]) / 32768.0f * 180.0f;
                    IMU->yaw   = (int16_t)((IMU->Buffer[7]<<8)|IMU->Buffer[6]) / 32768.0f * 180.0f;
                    WTIMU63_UpdateYaw(IMU);
                    memcpy(wimu_last_angle_buf, IMU->Buffer, 11);
                }
                else if(type == MAG)
                {
                    IMU->x_mag = (int16_t)((IMU->Buffer[3]<<8)|IMU->Buffer[2]) / 32768.0f * 180.0f;
                    IMU->y_mag = (int16_t)((IMU->Buffer[5]<<8)|IMU->Buffer[4]) / 32768.0f * 180.0f;
                    IMU->z_mag = (int16_t)((IMU->Buffer[7]<<8)|IMU->Buffer[6]) / 32768.0f * 180.0f;
                }
            }
            IMU->State   = WTIMU63_HEADER_PENDING;
            IMU->buf_idx = 0;
        }
        else
        {
            wimu_cs_fail++;
            wimu_consec_pass = 0;

            uint8_t resynced = 0;
            for(int i = 1; i < 11; i++)
            {
                if(IMU->Buffer[i] == 0x55)
                {
                    uint8_t remaining = 11 - i;
                    memmove(IMU->Buffer, &IMU->Buffer[i], remaining);
                    IMU->buf_idx = remaining;

                    if(remaining >= 2 && IMU->Buffer[1] >= 0x50 && IMU->Buffer[1] <= 0x5F)
                        IMU->State = WTIMU63_DATA_PENDING;
                    else if(remaining == 1)
                        IMU->State = WTIMU63_TYPE_PENDING;
                    else
                    {
                        IMU->State   = WTIMU63_HEADER_PENDING;
                        IMU->buf_idx = 0;
                    }
                    resynced = 1;
                    break;
                }
            }
            if(!resynced)
            {
                IMU->State   = WTIMU63_HEADER_PENDING;
                IMU->buf_idx = 0;
            }
        }

        HAL_UART_Receive_IT(IMU->huartx, &IMU->rx_byte, 1);
        break;
    }
}

/*
 * Function Name		: WTIMU63_I2CHandle
 * Function Description	: Called to handle completed I2C interrupt transfer.
 *						  Decodes 18 bytes read from registers 0x34 to 0x45:
 *						    Bytes  0- 5  : AX, AY, AZ     (int16 little-endian)
 *						    Bytes  6-11  : GX, GY, GZ     (int16 little-endian)
 *						    Bytes 12-17  : Roll, Pitch, Yaw (int16 little-endian)
 *						  Updates real_z and real_zrad for continuous yaw tracking.
 * Function Remarks		: Call inside HAL_I2C_MemRxCpltCallback() only when
 *						  hi2c->Instance matches IMU->hi2cimu->Instance.
 *						: Read raw register so no need checksum
 * Function Arguments	: *IMU
 * Function Return		: None
 * Function Example		: void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
 *						      if(hi2c->Instance == IMU.hi2cimu->Instance)
 *						          WTIMU63_I2CHandle(&IMU);
 *						  }
 */

void WTIMU63_I2CHandle(WTIMU63_t *IMU)
{
	IMU->x_acc = *((int16_t *)&IMU->Buffer[0])  / 32768.0f * 16.0f * 9.8f;
	IMU->y_acc = *((int16_t *)&IMU->Buffer[2])  / 32768.0f * 16.0f * 9.8f;
	IMU->z_acc = *((int16_t *)&IMU->Buffer[4])  / 32768.0f * 16.0f * 9.8f;

	IMU->roll_rate  = *((int16_t *)&IMU->Buffer[6])  / 32768.0f * 2000.0f;
	IMU->pitch_rate = *((int16_t *)&IMU->Buffer[8])  / 32768.0f * 2000.0f;
	IMU->yaw_rate   = *((int16_t *)&IMU->Buffer[10]) / 32768.0f * 2000.0f;

	IMU->roll  = *((int16_t *)&IMU->Buffer[12]) / 32768.0f * 180.0f;
	IMU->pitch = *((int16_t *)&IMU->Buffer[14]) / 32768.0f * 180.0f;
	IMU->yaw   = *((int16_t *)&IMU->Buffer[16]) / 32768.0f * 180.0f;

	WTIMU63_UpdateYaw(IMU);

	memcpy(wimu_i2c_raw, IMU->Buffer, 18);
	memset(IMU->Buffer, 0, 18);
	HAL_I2C_Mem_Read_IT(IMU->hi2cimu,(0x50 << 1),0x34,I2C_MEMADD_SIZE_8BIT, (uint8_t *)IMU->Buffer, 18);
}


/*
 * Function Name		: WTIMU63_DMAHandle
 * Function Description	: Called to handle completed I2C DMA transfer.
 *						  Samw decode logic to WTIMU63_I2CHandle.
 * Function Remarks		: Call inside HAL_I2C_MemRxCpltCallback() only when
 *						  hi2c->Instance matches IMU->hi2cimu->Instance.
 *						  Use this version when DMA mode was chosen in Init.
 * Function Arguments	: *IMU
 * Function Return		: None
 * Function Example		: void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
 *						      if(hi2c->Instance == IMU.hi2cimu->Instance)
 *						          WTIMU63_DMAHandle(&IMU);
 *						  }
 */
void WTIMU63_DMAHandle(WTIMU63_t *IMU)
{
	IMU->x_acc = *((int16_t *)&IMU->Buffer[0])  / 32768.0f * 16.0f * 9.8f;
	IMU->y_acc = *((int16_t *)&IMU->Buffer[2])  / 32768.0f * 16.0f * 9.8f;
	IMU->z_acc = *((int16_t *)&IMU->Buffer[4])  / 32768.0f * 16.0f * 9.8f;

	IMU->roll_rate  = *((int16_t *)&IMU->Buffer[6])  / 32768.0f * 2000.0f;
	IMU->pitch_rate = *((int16_t *)&IMU->Buffer[8])  / 32768.0f * 2000.0f;
	IMU->yaw_rate   = *((int16_t *)&IMU->Buffer[10]) / 32768.0f * 2000.0f;

	IMU->roll  = *((int16_t *)&IMU->Buffer[12]) / 32768.0f * 180.0f;
	IMU->pitch = *((int16_t *)&IMU->Buffer[14]) / 32768.0f * 180.0f;
	IMU->yaw   = *((int16_t *)&IMU->Buffer[16]) / 32768.0f * 180.0f;

	if(IMU->yaw < -150.0f){
		if(IMU->prev_yaw > 150.0f) IMU->yaw_constant++;
	}else if(IMU->yaw > 150.0f){
		if(IMU->prev_yaw < -150.0f) IMU->yaw_constant--;
	}
	IMU->prev_yaw	= IMU->yaw;
	IMU->real_z		= IMU->yaw + IMU->yaw_constant * 360.0f + IMU->offset;
	IMU->real_zrad	= (IMU->real_z / 180.0f) * 3.141593f;

	memset(IMU->Buffer, 0, 18);
	HAL_I2C_Mem_Read_IT(IMU->hi2cimu,(0x50 << 1),0x34,I2C_MEMADD_SIZE_8BIT, (uint8_t *)IMU->Buffer, 18);

}


void WTIMU63_SetOffset(WTIMU63_t *IMU, float offset)
{
    IMU->offset = offset;
    /* Immediately recalculate real_z with the new offset */
    IMU->real_z    = IMU->yaw + IMU->yaw_constant * 360.0f + IMU->offset;
    IMU->real_zrad = (IMU->real_z / 180.0f) * 3.141593f;
}

void WTIMU63_ResetYawTracking(WTIMU63_t *IMU)
{
    IMU->yaw_constant = 0.0f;
    IMU->prev_yaw     = 0.0f;
    IMU->real_z       = IMU->yaw + IMU->offset;
    IMU->real_zrad    = (IMU->real_z / 180.0f) * 3.141593f;
}

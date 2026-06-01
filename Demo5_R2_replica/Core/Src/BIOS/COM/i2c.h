/**
 * @file    i2c.h
 * @brief   header file for i2c driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Communication
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "../../Platform/H7_system.h"

//------------------- Enumeration -------------------//
typedef enum{
	CLK_SPEED_100KHz,
	CLK_SPEED_400KHz,
	CLK_SPEED_1MHz
} H7_i2c_speed;


//------------------- Structures -------------------//
typedef struct{
	I2C_HandleTypeDef *hi2c;
	DMA_HandleTypeDef hdma_i2c_rx;
	DMA_HandleTypeDef hdma_i2c_tx;
	u8 *rx_buffer;
	u8 *tx_buffer;
	u16 buffer_size;
	H7_state_e status;
} H7_i2cHandler_s;


//------------------- Macros and Global Variables -------------------//

//** Macros **//
#define main_board_1						8
#define main_board_2						9
#define main_board_3						10
#define main_board_4						11


//** Global variables **//


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c5;

extern H7_i2cHandler_s h7i2c1;
extern H7_i2cHandler_s h7i2c5;


//------------------- Function Declaration -------------------//

H7_state_e H7_i2cx_init_struct(H7_i2cHandler_s *i2c, I2C_HandleTypeDef *hi2c);
H7_state_e H7_i2cx_init(H7_i2cHandler_s *i2c, u32 ownAdress, u32 addressingMode, H7_i2c_speed clockSpeed);
H7_state_e H7_i2cx_DMA_RX_init(H7_i2cHandler_s *i2c, u32 DMA_mode, u32 ownAdress, u32 addressingMode, H7_i2c_speed clockSpeed);
H7_state_e H7_i2cx_DMA_TX_init(H7_i2cHandler_s *i2c, u32 DMA_mode, u32 ownAdress, u32 addressingMode, H7_i2c_speed clockSpeed);



#endif /* SRC_I2C_H_ */

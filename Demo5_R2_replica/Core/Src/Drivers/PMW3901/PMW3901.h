/************************************************


**************************************
 * 		Include Libraries 			   *
 **************************************/
#ifndef PMW3901_H_
#define PMW3901_H_

#include "../../BIOS/BIOS.h"
#include "../../BIOS/COM/i2c.h"

/************************************************/
/*		    	Define         					*/
/************************************************/


/************************************************/
/*				Struct         					*/
/************************************************/

typedef struct{

	I2C_HandleTypeDef* hi2c;
	DMA_HandleTypeDef* hi2c_rx_dma;
	UART_HandleTypeDef* huart;

	uint8_t ReceiveBuffer[20];

	int16_t input_x;
	int16_t input_y;

	float x;
	float y;

	uint8_t slave;
	uint8_t master;

	uint8_t disconnected;


}PMW3901_t;

/**************************************************
 * 		STRUCTURE DEFINES					  	  *
 *************************************************/

/**************************************************
 * 		Function Prototype					  	  *
 *************************************************/

void PMW3901_SlaveInit(PMW3901_t *PMW3901, H7_i2cHandler_s *i2c);
void PMW3901_SlaveHandler(PMW3901_t *PMW3901);
void PMW3901_ConnectionHandler(PMW3901_t *PMW3901, H7_i2cHandler_s *i2c);
void PMW3901_GetXY(PMW3901_t *PMW3901);

#endif /* PSX_INTERFACE_PSX_INTERFACE_H_ */

/**
 * @file    i2c.c
 * @brief   i2c driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Communication
 */

//** Includes **//

#include "i2c.h"



//** Global variables **//
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c5;

H7_i2cHandler_s h7i2c1;
H7_i2cHandler_s h7i2c5;



//------------------- Function Defination -------------------//



H7_state_e H7_i2cx_init_struct(H7_i2cHandler_s *i2c, I2C_HandleTypeDef *hi2c){
	if(i2c == NULL || hi2c == NULL){
		return H7_I2C_NULL_PTR;
	}
	i2c->hi2c	= hi2c;
	i2c->status	= H7_PERIPH_OK;
	return H7_PERIPH_OK;
}
H7_state_e H7_i2cx_init(H7_i2cHandler_s *i2c, u32 ownAdress, u32 addressingMode, H7_i2c_speed clockSpeed){

	u32 analog_filter = I2C_ANALOGFILTER_ENABLE;
	u32 fastModePlus;
	if(i2c->hi2c == &hi2c1){
		i2c->hi2c->Instance = I2C1;
		fastModePlus 		= I2C_FASTMODEPLUS_I2C1;
	}else if(i2c->hi2c == &hi2c5){
		i2c->hi2c->Instance = I2C5;
		fastModePlus 		= I2C_FASTMODEPLUS_I2C5;
	}else{
		i2c->status	= H7_I2C_INVALID_PORT;
		return H7_I2C_INVALID_PORT;
	}
	switch(clockSpeed){
	case CLK_SPEED_100KHz:
		i2c->hi2c->Init.Timing = 0x60404E72;
		break;
	case CLK_SPEED_400KHz:
		i2c->hi2c->Init.Timing = 0x00D049FB;
		break;
	case CLK_SPEED_1MHz:
		i2c->hi2c->Init.Timing = 0x00601A5C;
		analog_filter = I2C_ANALOGFILTER_DISABLE;
		break;
	default:
		i2c->status	= H7_I2C_INVALID_SPEED;
		return H7_I2C_INVALID_SPEED;
	}
	i2c->hi2c->Init.OwnAddress1 		= (ownAdress << 1);
	i2c->hi2c->Init.AddressingMode 		= addressingMode;
	i2c->hi2c->Init.DualAddressMode 	= I2C_DUALADDRESS_DISABLE;
	i2c->hi2c->Init.OwnAddress2 		= 0;
	i2c->hi2c->Init.OwnAddress2Masks 	= I2C_OA2_NOMASK;  // Masking address which allows to respond to multiple addresses
	i2c->hi2c->Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
	i2c->hi2c->Init.NoStretchMode 		= I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(i2c->hi2c) != HAL_OK)
	{
		i2c->status	= H7_I2C_INIT_ERR;
		return H7_I2C_INIT_ERR;
//	    Error_Handler();
	}

	  /** Configure Analogue filter
	   */

	if (HAL_I2CEx_ConfigAnalogFilter(i2c->hi2c, analog_filter) != HAL_OK)
	{
		i2c->status	= H7_I2C_AN_FILTER_CONFIG_ERR;
		return H7_I2C_AN_FILTER_CONFIG_ERR;
//		Error_Handler();
	}

	  /** Configure Digital filter
	   * glitch filter
	   * Second argument is a value between 0 and 15
	   * 0 to disable the filter
	   * 15 is the maximum filtering
	   * Use when have High noise environment
	  */
	if (HAL_I2CEx_ConfigDigitalFilter(i2c->hi2c, 0) != HAL_OK)
	{
		i2c->status	= H7_I2C_DG_FILTER_CONFIG_ERR;
		return H7_I2C_DG_FILTER_CONFIG_ERR;
//		Error_Handler();
	}
	if(clockSpeed == CLK_SPEED_1MHz){
		HAL_I2CEx_EnableFastModePlus(fastModePlus);
	}
	i2c->status	= H7_PERIPH_OK;
	return H7_PERIPH_OK;
}




H7_state_e H7_i2cx_DMA_RX_init(H7_i2cHandler_s *i2c, u32 DMA_mode, u32 ownAdress, u32 addressingMode, H7_i2c_speed clockSpeed){

	u32 analog_filter = I2C_ANALOGFILTER_ENABLE;
	u32 fastModePlus;
	if(i2c->hi2c == &hi2c1){
		i2c->hi2c->Instance 			= I2C1;
		i2c->hdma_i2c_rx.Instance		= DMA2_Stream0;
		i2c->hdma_i2c_rx.Init.Request 	= DMA_REQUEST_I2C1_RX;
		HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
		fastModePlus 		= I2C_FASTMODEPLUS_I2C1;
	}else if(i2c->hi2c == &hi2c5){
		i2c->hi2c->Instance = I2C5;
		i2c->hdma_i2c_rx.Instance		= DMA2_Stream2;
		i2c->hdma_i2c_rx.Init.Request 	= DMA_REQUEST_I2C5_RX;
		HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		fastModePlus 		= I2C_FASTMODEPLUS_I2C5;
	}else{
		i2c->status = H7_I2C_INVALID_PORT;
		return H7_I2C_INVALID_PORT;
	}
	switch(clockSpeed){
	case CLK_SPEED_100KHz:
		i2c->hi2c->Init.Timing = 0x60404E72;
		break;
	case CLK_SPEED_400KHz:
		i2c->hi2c->Init.Timing = 0x00D049FB;
		break;
	case CLK_SPEED_1MHz:
		i2c->hi2c->Init.Timing = 0x00601A5C;
		analog_filter = I2C_ANALOGFILTER_DISABLE;
		break;
	default:
		i2c->status = H7_I2C_INVALID_SPEED;
		return H7_I2C_INVALID_SPEED;
	}
	i2c->hi2c->Init.OwnAddress1 		= (ownAdress << 1);
	i2c->hi2c->Init.AddressingMode 		= addressingMode;
	i2c->hi2c->Init.DualAddressMode 	= I2C_DUALADDRESS_DISABLE;
	i2c->hi2c->Init.OwnAddress2 		= 0;
	i2c->hi2c->Init.OwnAddress2Masks 	= I2C_OA2_NOMASK;  // Masking address which allows to respond to multiple addresses
	i2c->hi2c->Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
	i2c->hi2c->Init.NoStretchMode 		= I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(i2c->hi2c) != HAL_OK)
	{
		i2c->status = H7_I2C_INIT_ERR;
		return H7_I2C_INIT_ERR;
//	    Error_Handler();
	}

	// DMA Configuration
	i2c->hdma_i2c_rx.Init.Direction				= DMA_PERIPH_TO_MEMORY;
	i2c->hdma_i2c_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	i2c->hdma_i2c_rx.Init.MemInc 				= DMA_MINC_ENABLE;
	i2c->hdma_i2c_rx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
	i2c->hdma_i2c_rx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
	i2c->hdma_i2c_rx.Init.Mode 					= DMA_mode;
	i2c->hdma_i2c_rx.Init.Priority 				= DMA_PRIORITY_HIGH;
	i2c->hdma_i2c_rx.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	i2c->hdma_i2c_rx.Init.FIFOThreshold 		= DMA_FIFO_THRESHOLD_1QUARTERFULL;
	i2c->hdma_i2c_rx.Init.MemBurst 				= DMA_MBURST_SINGLE;
	i2c->hdma_i2c_rx.Init.PeriphBurst 			= DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&i2c->hdma_i2c_rx) != HAL_OK)
    {
    	i2c->status	= H7_I2C_DMA_INIT_ERR;
    	return H7_I2C_DMA_INIT_ERR;
//      Error_Handler();
    }
    __HAL_LINKDMA(i2c->hi2c,hdmarx, i2c->hdma_i2c_rx);
	  /** Configure Analogue filter
	   */

	if (HAL_I2CEx_ConfigAnalogFilter(i2c->hi2c, analog_filter) != HAL_OK)
	{
		i2c->status = H7_I2C_AN_FILTER_CONFIG_ERR;
		return H7_I2C_AN_FILTER_CONFIG_ERR;
//		Error_Handler();
	}

	  /** Configure Digital filter
	   * glitch filter
	   * Second argument is a value between 0 and 15
	   * 0 to disable the filter
	   * 15 is the maximum filtering
	   * Use when have High noise environment
	  */
	if (HAL_I2CEx_ConfigDigitalFilter(i2c->hi2c, 0) != HAL_OK)
	{
		i2c->status = H7_I2C_DG_FILTER_CONFIG_ERR;
		return H7_I2C_DG_FILTER_CONFIG_ERR;
//		Error_Handler();
	}
	if(clockSpeed == CLK_SPEED_1MHz){
		HAL_I2CEx_EnableFastModePlus(fastModePlus);
	}
	i2c->status = H7_PERIPH_OK;
	return H7_PERIPH_OK;
}

H7_state_e H7_i2cx_DMA_TX_init(H7_i2cHandler_s *i2c, u32 DMA_mode, u32 ownAdress, u32 addressingMode, H7_i2c_speed clockSpeed){

	u32 analog_filter = I2C_ANALOGFILTER_ENABLE;
	u32 fastModePlus;
	if(i2c->hi2c == &hi2c1){
		i2c->hi2c->Instance 			= I2C1;
		i2c->hdma_i2c_tx.Instance		= DMA2_Stream1;
		i2c->hdma_i2c_tx.Init.Request 	= DMA_REQUEST_I2C1_TX;
		HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		fastModePlus 		= I2C_FASTMODEPLUS_I2C1;
	}else if(i2c->hi2c == &hi2c5){
		i2c->hi2c->Instance 			= I2C5;
		i2c->hdma_i2c_tx.Instance		= DMA2_Stream3;
		i2c->hdma_i2c_tx.Init.Request 	= DMA_REQUEST_I2C5_TX;
		HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
		fastModePlus 		= I2C_FASTMODEPLUS_I2C5;
	}else{
		i2c->status = H7_I2C_INVALID_PORT;
		return H7_I2C_INVALID_PORT;
	}
	switch(clockSpeed){
	case CLK_SPEED_100KHz:
		i2c->hi2c->Init.Timing = 0x60404E72;
		break;
	case CLK_SPEED_400KHz:
		i2c->hi2c->Init.Timing = 0x00D049FB;
		break;
	case CLK_SPEED_1MHz:
		i2c->hi2c->Init.Timing = 0x00601A5C;
		analog_filter = I2C_ANALOGFILTER_DISABLE;
		break;
	default:
		i2c->status = H7_I2C_INVALID_SPEED;
		return H7_I2C_INVALID_SPEED;
	}
	i2c->hi2c->Init.OwnAddress1 		= (ownAdress << 1);
	i2c->hi2c->Init.AddressingMode 		= addressingMode;
	i2c->hi2c->Init.DualAddressMode 	= I2C_DUALADDRESS_DISABLE;
	i2c->hi2c->Init.OwnAddress2 		= 0;
	i2c->hi2c->Init.OwnAddress2Masks 	= I2C_OA2_NOMASK;  // Masking address which allows to respond to multiple addresses
	i2c->hi2c->Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
	i2c->hi2c->Init.NoStretchMode 		= I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(i2c->hi2c) != HAL_OK)
	{
		i2c->status = H7_I2C_INIT_ERR;
		return H7_I2C_INIT_ERR;
//	    Error_Handler();
	}

	// DMA Configuration
	i2c->hdma_i2c_rx.Init.Direction				= DMA_PERIPH_TO_MEMORY;
	i2c->hdma_i2c_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	i2c->hdma_i2c_rx.Init.MemInc 				= DMA_MINC_ENABLE;
	i2c->hdma_i2c_rx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
	i2c->hdma_i2c_rx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
	i2c->hdma_i2c_rx.Init.Mode 					= DMA_mode;
	i2c->hdma_i2c_rx.Init.Priority 				= DMA_PRIORITY_HIGH;
	i2c->hdma_i2c_rx.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	i2c->hdma_i2c_rx.Init.FIFOThreshold 		= DMA_FIFO_THRESHOLD_1QUARTERFULL;
	i2c->hdma_i2c_rx.Init.MemBurst 				= DMA_MBURST_SINGLE;
	i2c->hdma_i2c_rx.Init.PeriphBurst 			= DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&i2c->hdma_i2c_rx) != HAL_OK)
    {
    	i2c->status	= H7_I2C_DMA_INIT_ERR;
    	return H7_I2C_DMA_INIT_ERR;
//      Error_Handler();
    }
    __HAL_LINKDMA(i2c->hi2c,hdmarx, i2c->hdma_i2c_rx);
	  /** Configure Analogue filter
	   */

	if (HAL_I2CEx_ConfigAnalogFilter(i2c->hi2c, analog_filter) != HAL_OK)
	{
		i2c->status = H7_I2C_AN_FILTER_CONFIG_ERR;
		return H7_I2C_AN_FILTER_CONFIG_ERR;
//		Error_Handler();
	}

	  /** Configure Digital filter
	   * glitch filter
	   * Second argument is a value between 0 and 15
	   * 0 to disable the filter
	   * 15 is the maximum filtering
	   * Use when have High noise environment
	  */
	if (HAL_I2CEx_ConfigDigitalFilter(i2c->hi2c, 0) != HAL_OK)
	{
		i2c->status = H7_I2C_DG_FILTER_CONFIG_ERR;
		return H7_I2C_DG_FILTER_CONFIG_ERR;
//		Error_Handler();
	}
	if(clockSpeed == CLK_SPEED_1MHz){
		HAL_I2CEx_EnableFastModePlus(fastModePlus);
	}
	i2c->status = H7_PERIPH_OK;
	return H7_PERIPH_OK;
}


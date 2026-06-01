/**
 * @file    callback.c
 * @brief   The file contains all callback functions
 * @author  Mohammed Abdulalem
 * @date    2026-2-12
 *
 *
 * @note
 */

#include "callbacks.h"

extern int test1;
// R2 code
extern osSemaphoreId_t fivemsSemaphore;
extern volatile uint8_t tx_busy;
extern double timer;
extern uint8_t rxChar;
extern volatile uint8_t msg_received;
extern char rxBuffer[100];
extern volatile uint32_t tim13_cnt;

// Laptop -> H7 raw state (defined in main.c, see r2_world_state.h)
#include "r2_world_state.h"
extern volatile R2RawState_t g_raw;

uint32_t can_running = 0;
//------------------- Tiemr Callbacks -------------------//


/**
 * @brief  Period elapsed callback in non blocking mode
 * @note	Use volatile variables for best performance
 * @param  htim : TIM handle
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1){

	}else if(htim->Instance == TIM3){
		Path_Profile_Inc();
	}
	else if(htim->Instance == TIM4){

	}

	else if (htim->Instance == TIM6)
	{
		HAL_IncTick();
	}

	else if (htim->Instance == TIM7) // 5ms
	{

		myswerve.timer++;
		timer += 0.005;
		static uint8_t led = 0;
		if (++led > 4) { //20ms
			led2 = !led2;
			PSxConnectionHandler(&ps4);
			led = 0;
		}

	}else if(htim->Instance == TIM12){ // 5 ms
		osSemaphoreRelease(fivemsSemaphore);
		//	    osThreadFlagsSet(fivemsTaskHandle, 0x0001);
		tim13_cnt++;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1) {
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)){
			QEI1.count--;
		}
		else {
			QEI1.count++;
		}
	}

	if (htim->Instance == TIM8) {
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8)){
			QEI2.count--;
		}
		else {
			QEI2.count++;
		}
	}

}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_TriggerCallback could be implemented in the user file
	 */
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_ErrorCallback could be implemented in the user file
	 */
}


//------------------- UART Callbacks -------------------//

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t rxIndex = 0;
	if (huart == IMU63.huartx) {
				// IMU_Handler(&IMU);
		WTIMU63_Handler(&IMU63);
	}
	else if(huart == tfmini.huartx){
		TFmini_Handler(&tfmini);
	}
	else if(huart->Instance == UART9){
		//		HAL_UART_Receive_IT(&huart9, &buff, 1);
	}
	else if(huart->Instance == UART5){

	}
	else if(huart->Instance == UART7){
		//		HAL_UART_Receive_IT(&huart7, &buff, 1);
	}
	else if(huart->Instance == UART4){
		//		HAL_UART_Receive_IT(&huart5, &buff, 1);
		HAL_UART_Receive_IT(&huart5, &rxChar, 1); // re-arm
		// End-of-line detection
		if (rxChar == '\n')  // handle '\n' only
		{
			if (rxIndex > 0)
			{
				rxBuffer[rxIndex] = '\0';
				msg_received = 1;  // signal task
				rxIndex = 0;
			}
			return;
		}

		// Store character if there is space
		if (rxIndex < sizeof(rxBuffer) - 1)
			rxBuffer[rxIndex++] = rxChar;
		else
		{
			rxIndex = 0; // overflow, reset buffer
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5) {
		tx_busy = 0;   // mark TX finished
	}
}

//------------------- ADC Callbacks -------------------//


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/* Prevent unused argument(s) compilation warning */
	//  UNUSED(hadc);

	if(hadc->Instance == ADC1){

	}else if(hadc->Instance == ADC2){

	}else if(hadc->Instance == ADC3){

	}

	/* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
	 */
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	/* Prevent unused argument(s) compilation warning */
	//  UNUSED(hadc);
	if(hadc->Instance == ADC1){

	}else if(hadc->Instance == ADC2){

	}else if(hadc->Instance == ADC3){

	}
	/* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ErrorCallback must be implemented in the user file.
	 */
}



//------------------- I2C Callbacks -------------------//

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C5){
		TF_Bus_TxCpltCallback(&i2c5_bus, hi2c);
		//		TF_I2C_TxCpltCallback(&TF_NOVA, hi2c);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2c);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterRxCpltCallback could be implemented in the user file
	 */
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2c);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_SlaveTxCpltCallback could be implemented in the user file
	 */
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == ps4.h7i2cps4->hi2c){
		PSx_SlaveHandler(&ps4);
	}
	//	else if(hi2c==enc.hi2c){
	//		PMW3901_SlaveHandler(&enc);
	//	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2c);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MemTxCpltCallback could be implemented in the user file
	 */
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2c);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MemRxCpltCallback could be implemented in the user file
	 */
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C5){
		TF_Bus_ErrorCallback(&i2c5_bus, hi2c);
	}
}



//------------------- FDCAN Callbacks -------------------//

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	PACKET_t source = 0;
	//** FDCAN1 **//
	if(hfdcan->Instance == FDCAN1){
		if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &h7fdcan1.rxHeader, h7fdcan1.rxData) != HAL_OK){
			Error_Handler();
		}
		// Handle Extended IDs
		if(h7fdcan1.rxHeader.IdType == FDCAN_EXTENDED_ID){ // VESc and cyberGear
			if ((h7fdcan1.rxHeader.Identifier & 0xFF) >= 112 && (h7fdcan1.rxHeader.Identifier & 0xFF) < 127) {
				vescmsg.Rxmsg = h7fdcan1.rxHeader;
				memcpy(vescmsg.Data, h7fdcan1.rxData, 8);
				set_rx_frames(&vescmsg);
				source = VESC_PACKET;
			} else {
				Cybergear_FDCAN_Handler(&cybergear1, &h7fdcan1.rxHeader, h7fdcan1.rxData);
				source = CYBERGEAR_PACKET;
			}
		}
		// Standard IDs
		else if(h7fdcan1.rxHeader.IdType == FDCAN_STANDARD_ID){
			uint8_t j60_cmd = (h7fdcan1.rxHeader.Identifier >> 5);
			uint8_t j60_id 	= (h7fdcan1.rxHeader.Identifier & 0x1F);

			if(h7fdcan1.rxHeader.Identifier >= 0x201 && h7fdcan1.rxHeader.Identifier <= 0x208){
				RBMS_CAN_Handler(&h7fdcan1.rxHeader, h7fdcan1.rxData, 1);
			}
			else if (j60_id == J60_1.id) {
				if (j60_cmd == J60_CMD_CONTROL) {
					J60_ParseFeedback(&J60_1, h7fdcan1.rxData, h7fdcan1.rxHeader.Identifier);
				}
				else if (j60_cmd == J60_CMD_GET_STATUS) {
					J60_1.error_code = (h7fdcan1.rxData[0] | (h7fdcan1.rxData[1] << 8));
				}
			}
		}
		CAN_PROCESS(source);

		if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
			Error_Handler();
		}
	}



	//** FDCAN2 **//
	if(hfdcan->Instance == FDCAN2){

		if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &h7fdcan2.rxHeader, h7fdcan2.rxData) != HAL_OK){
			Error_Handler();
		}
		// Handle Extended IDs
		if(h7fdcan2.rxHeader.IdType == FDCAN_EXTENDED_ID){ // VESc and cyberGear
			if ((h7fdcan2.rxHeader.Identifier & 0xFF) >= 112 && (h7fdcan2.rxHeader.Identifier & 0xFF) < 127) {
				vescmsg.Rxmsg = h7fdcan2.rxHeader;
				memcpy(vescmsg.Data, h7fdcan2.rxData, 8);
				set_rx_frames(&vescmsg);
				source = VESC_PACKET;
			}else {
				Cybergear_FDCAN_Handler(&cybergear1, &h7fdcan2.rxHeader, h7fdcan2.rxData);
				source = CYBERGEAR_PACKET;
			}
		}else if(h7fdcan2.rxHeader.IdType == FDCAN_STANDARD_ID){
			uint8_t j60_cmd = (h7fdcan2.rxHeader.Identifier >> 5);
			uint8_t j60_id 	= (h7fdcan2.rxHeader.Identifier & 0x1F);


			if (j60_id == J60_1.id) {

				// Direct the data based on the command type
				if (j60_cmd == J60_CMD_CONTROL) {
					// This updates position, velocity, torque, and temp
					J60_ParseFeedback(&J60_1, h7fdcan2.rxData, h7fdcan2.rxHeader.Identifier);
				}
				else if (j60_cmd == J60_CMD_GET_STATUS) {
					// Specifically handle the status/error word
					J60_1.error_code = (h7fdcan2.rxData[0] | (h7fdcan2.rxData[1] << 8));
					// You can also extract voltage/current here if needed
				}
			}
			else if(h7fdcan2.rxHeader.Identifier >= 0x201 && h7fdcan2.rxHeader.Identifier <= 0x208){
				RBMS_CAN_Handler(&h7fdcan2.rxHeader, h7fdcan2.rxData, 1);

			}
		}
		CAN_PROCESS(source);

		if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
			Error_Handler();
		}
	}


	//** FDCAN3 **//
	if(hfdcan->Instance == FDCAN3){

		if(HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &h7fdcan3.rxHeader, h7fdcan3.rxData) != HAL_OK){
			Error_Handler();
		}

		// Handle Extended IDs
		if(h7fdcan3.rxHeader.IdType == FDCAN_EXTENDED_ID){ // VESc and cyberGear
			if ((h7fdcan3.rxHeader.Identifier & 0xFF) >= 112 && (h7fdcan3.rxHeader.Identifier & 0xFF) < 127) {
				vescmsg.Rxmsg = h7fdcan3.rxHeader;
				memcpy(vescmsg.Data, h7fdcan3.rxData, 8);
				set_rx_frames(&vescmsg);
				source = VESC_PACKET;
			}else {
				Cybergear_FDCAN_Handler(&cybergear1, &h7fdcan3.rxHeader, h7fdcan3.rxData);
				source = CYBERGEAR_PACKET;
			}

			// Handle Standard IDs
		}else if(h7fdcan3.rxHeader.IdType == FDCAN_STANDARD_ID){
			uint32_t std_id = h7fdcan3.rxHeader.Identifier;

			// Laptop -> H7 frames first (avoid colliding with J60 ID space)
			if(std_id == CAN_ID_R2_PATH){
				can_running++;
				// TODO: route into r2can multi-frame parser
			}
			else if(std_id == CAN_ID_FOREST1){
				can_running++;
				const R2_Forest1Frame_t *f = (const R2_Forest1Frame_t *)h7fdcan3.rxData;
				memcpy((void*)g_raw.forest, f->blocks, 8);
			}
			else if(std_id == CAN_ID_FOREST2){
				can_running++;
				const R2_Forest2Frame_t *f = (const R2_Forest2Frame_t *)h7fdcan3.rxData;
				g_raw.forest[8]     = f->blocks[0];
				g_raw.forest[9]     = f->blocks[1];
				g_raw.forest[10]    = f->blocks[2];
				g_raw.forest[11]    = f->blocks[3];
				g_raw.current_block = f->current_block;
				g_raw.status_flags  = f->status_flags;
			}
			else if(std_id == CAN_ID_R1_STATUS){
				can_running++;
				const R2_R1StatusFrame_t *f = (const R2_R1StatusFrame_t *)h7fdcan3.rxData;
				g_raw.r1_pose_x_mm = (int16_t)__builtin_bswap16((uint16_t)f->x_mm_be);
				g_raw.r1_pose_y_mm = (int16_t)__builtin_bswap16((uint16_t)f->y_mm_be);
				g_raw.r1_zone      = f->zone;
			}
			else if(std_id == CAN_ID_R2_POSITION){
				can_running++;
				const R2_PoseFrame_t *f = (const R2_PoseFrame_t *)h7fdcan3.rxData;
				g_raw.r2_pose_x_mm = f->x_mm;
				g_raw.r2_pose_y_mm = f->y_mm;
				g_raw.r2_pose_z_mm = f->z_mm;
				g_raw.r2_yaw_cdeg  = f->yaw_cdeg;
			}else if(std_id == 0x109){ // Latency test ID
				can_running++;
				FDCAN_TxMsg(&hfdcan3, 0x10A, h7fdcan3.rxData, 8); // reply with next ID to signal receipt
			}
			else {
				uint8_t j60_cmd = (std_id >> 5);
				uint8_t j60_id  = (std_id & 0x1F);

				if (j60_id == J60_1.id) {
					if (j60_cmd == J60_CMD_CONTROL) {
						J60_ParseFeedback(&J60_1, h7fdcan3.rxData, std_id);
					}
					else if (j60_cmd == J60_CMD_GET_STATUS) {
						J60_1.error_code = (h7fdcan3.rxData[0] | (h7fdcan3.rxData[1] << 8));
					}
				}
				else if(std_id >= 0x201 && std_id <= 0x208){
					RBMS_CAN_Handler(&h7fdcan3.rxHeader, h7fdcan3.rxData, 3);
				}
			}
		}
		CAN_PROCESS(source);
		if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
			Error_Handler();
		}
	}
	// Check for FIFO overrun
	//        if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
	//        {
	//            can_errors++;
	//        }


}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	PACKET_t source = 0;

	//** FDCAN1 **//
	if(hfdcan->Instance == FDCAN1){
		if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &h7fdcan1.rxHeader, h7fdcan1.rxData) != HAL_OK){
			Error_Handler();
		}
		// Handle Extended IDs
		if(h7fdcan1.rxHeader.IdType == FDCAN_EXTENDED_ID){ // VESc and cyberGear
			if ((h7fdcan1.rxHeader.Identifier & 0xFF) >= 112 && (h7fdcan1.rxHeader.Identifier & 0xFF) < 127) {
				vescmsg.Rxmsg = h7fdcan1.rxHeader;
				memcpy(vescmsg.Data, h7fdcan1.rxData, 8);
				set_rx_frames(&vescmsg);
				source = VESC_PACKET;
			} else {
				Cybergear_FDCAN_Handler(&cybergear1, &h7fdcan1.rxHeader, h7fdcan1.rxData);
				source = CYBERGEAR_PACKET;
			}
		}else if(h7fdcan1.rxHeader.IdType == FDCAN_STANDARD_ID){
			uint8_t j60_cmd = (h7fdcan1.rxHeader.Identifier >> 5);
			uint8_t j60_id 	= (h7fdcan1.rxHeader.Identifier & 0x1F);


			if (j60_id == J60_1.id) {

				// Direct the data based on the command type
				if (j60_cmd == J60_CMD_CONTROL) {
					// This updates position, velocity, torque, and temp
					J60_ParseFeedback(&J60_1, h7fdcan1.rxData, h7fdcan1.rxHeader.Identifier);
				}
				else if (j60_cmd == J60_CMD_GET_STATUS) {
					// Specifically handle the status/error word
					J60_1.error_code = (h7fdcan1.rxData[0] | (h7fdcan1.rxData[1] << 8));
					// You can also extract voltage/current here if needed
				}
			}
			else if(h7fdcan1.rxHeader.Identifier >= 0x201 && h7fdcan1.rxHeader.Identifier <= 0x208){
				RBMS_CAN_Handler(&h7fdcan1.rxHeader, h7fdcan1.rxData, 1);

			}
		}
		CAN_PROCESS(source);

		if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
			Error_Handler();
		}
	}

	//** FDCAN2 **//
	if(hfdcan->Instance == FDCAN2){

		if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &h7fdcan2.rxHeader, h7fdcan2.rxData) != HAL_OK){
			Error_Handler();
		}
		// Handle Extended IDs
		if(h7fdcan2.rxHeader.IdType == FDCAN_EXTENDED_ID){ // VESc and cyberGear
			if ((h7fdcan2.rxHeader.Identifier & 0xFF) >= 112 && (h7fdcan2.rxHeader.Identifier & 0xFF) < 127) {
				vescmsg.Rxmsg = h7fdcan2.rxHeader;
				memcpy(vescmsg.Data, h7fdcan2.rxData, 8);
				set_rx_frames(&vescmsg);
				source = VESC_PACKET;
			}else {
				Cybergear_FDCAN_Handler(&cybergear1, &h7fdcan2.rxHeader, h7fdcan2.rxData);
				source = CYBERGEAR_PACKET;
			}
		}else if(h7fdcan2.rxHeader.IdType == FDCAN_STANDARD_ID){
			uint8_t j60_cmd = (h7fdcan2.rxHeader.Identifier >> 5);
			uint8_t j60_id 	= (h7fdcan2.rxHeader.Identifier & 0x1F);


			if (j60_id == J60_1.id) {

				// Direct the data based on the command type
				if (j60_cmd == J60_CMD_CONTROL) {
					// This updates position, velocity, torque, and temp
					J60_ParseFeedback(&J60_1, h7fdcan2.rxData, h7fdcan2.rxHeader.Identifier);
				}
				else if (j60_cmd == J60_CMD_GET_STATUS) {
					// Specifically handle the status/error word
					J60_1.error_code = (h7fdcan2.rxData[0] | (h7fdcan2.rxData[1] << 8));
					// You can also extract voltage/current here if needed
				}
			}
			else if(h7fdcan2.rxHeader.Identifier >= 0x201 && h7fdcan2.rxHeader.Identifier <= 0x208){
				RBMS_CAN_Handler(&h7fdcan2.rxHeader, h7fdcan2.rxData, 1);

			}
		}
		CAN_PROCESS(source);

		if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
			Error_Handler();
		}
	}


	//** FDCAN3 **//
	if(hfdcan->Instance == FDCAN3){
		if(HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &h7fdcan3.rxHeader, h7fdcan3.rxData) != HAL_OK){
			Error_Handler();
		}
		// Handle Extended IDs
		if(h7fdcan3.rxHeader.IdType == FDCAN_EXTENDED_ID){ // VESc and cyberGear
			if ((h7fdcan3.rxHeader.Identifier & 0xFF) >= 112 && (h7fdcan3.rxHeader.Identifier & 0xFF) < 127) {
				vescmsg.Rxmsg = h7fdcan3.rxHeader;
				memcpy(vescmsg.Data, h7fdcan3.rxData, 8);
				set_rx_frames(&vescmsg);
				source = VESC_PACKET;
			}else {
				Cybergear_FDCAN_Handler(&cybergear1, &h7fdcan3.rxHeader, h7fdcan3.rxData);
				source = CYBERGEAR_PACKET;
			}

		}
		// Handle Standard IDs
		else if(h7fdcan3.rxHeader.IdType == FDCAN_STANDARD_ID){
			uint8_t j60_cmd = (h7fdcan3.rxHeader.Identifier >> 5);
			uint8_t j60_id 	= (h7fdcan3.rxHeader.Identifier & 0x1F);


			if (j60_id == J60_1.id) {

				// Direct the data based on the command type
				if (j60_cmd == J60_CMD_CONTROL) {
					// This updates position, velocity, torque, and temp
					J60_ParseFeedback(&J60_1, h7fdcan3.rxData, h7fdcan3.rxHeader.Identifier);
				}
				else if (j60_cmd == J60_CMD_GET_STATUS) {
					// Specifically handle the status/error word
					J60_1.error_code = (h7fdcan3.rxData[0] | (h7fdcan3.rxData[1] << 8));
					// You can also extract voltage/current here if needed
				}
			}
			else if(h7fdcan3.rxHeader.Identifier >= 0x201 && h7fdcan3.rxHeader.Identifier <= 0x208){
				RBMS_CAN_Handler(&h7fdcan3.rxHeader, h7fdcan3.rxData, 1);

			}
		}
		CAN_PROCESS(source);
		if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
			Error_Handler();
		}
	}
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorCodeITs)
{
	if (ErrorCodeITs & FDCAN_IT_BUS_OFF) {
		//        can_errors = 10;
	}
}



//------------------- EXTI Callbacks -------------------//
/**
 * @brief 	This callback for EXTI pins
 * @param	GPIO_Pin
 * @warning	This should be used carfully is a pin number shares all GPIO ports,
 * so select only one port for each oin number
 * @note	ADC pins are recomended to be used
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	// Handling ADC in the board Pins
	switch(GPIO_Pin) {
	case GPIO_PIN_0: // A5 GPIOB0
		// Handle PA0, PB0, PC0, etc.

		break;

	case GPIO_PIN_1: // A6 GPIOB1

		break;

	case GPIO_PIN_2: // A2 GPIOC2
		//        	PWMEncoder_Angle(&htim12, GPIOC, GPIO_PIN_2, &enc2);
		break;

	case GPIO_PIN_3: // A1 GPIOC3
		//        	PWMEncoder_Angle(&htim12, GPIOC, GPIO_PIN_3, &enc1);
		break;

	case GPIO_PIN_4: // A4 GPIOA4
		//        	PWMEncoder_Angle(&htim12, GPIOA, GPIO_PIN_4, &enc4);
		break;

	case GPIO_PIN_5: // A3 GPIOA5
		//        	PWMEncoder_Angle(&htim12, GPIOA, GPIO_PIN_5, &enc3);
		break;

	case GPIO_PIN_6:

		break;

	case GPIO_PIN_7:

		break;

	case GPIO_PIN_8:

		break;

	case GPIO_PIN_9:

		break;

	case GPIO_PIN_10:

		break;

	case GPIO_PIN_11:

		break;

	case GPIO_PIN_12:

		break;

	case GPIO_PIN_13:

		break;

	case GPIO_PIN_14:

		break;

	case GPIO_PIN_15:

		break;

	default:

		break;
	}
}




//------------------- SPI Callbacks -------------------//


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) /* Derogation MISRAC2012-Rule-8.13 */
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
	 */
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) /* Derogation MISRAC2012-Rule-8.13 */
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_ErrorCallback should be implemented in the user file
	 */
	/* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
            and user can use HAL_SPI_GetError() API to check the latest error occurred
	 */
}

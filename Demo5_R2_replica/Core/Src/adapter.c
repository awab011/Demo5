/**
 * @file    adapter.c
 * @brief   This file contain the internal driver initialization and ports
 * @author  Mohammed Abdulalem
 * @date    2026-2-12
 *
 * @details All ports should be configured here
 *
 * @note
 */


#include "adapter.h"


// Global Device Structures

// Timers HAL handlers


// RBC Libraries global variables
tfmini_t tfmini;
R6091U_t IMU;
PSxBT_t ps4;
H7_EXTIHandler_s EXTI_Handler;
PWMEnc_t enc1,enc2,enc3,enc4;
H7_state_e rslt;

Path_Profile_t climb_prof,descend_prof,cbg_prof,j60_prof;
Navigator_t path_plan;
Navigator_t mc_pp;
Navigator_t mf_pp;

float x_vel, y_vel, w_vel, v1, v2, v3, v4, d1, d2, d3, d4;         //MODN variables
float angle_error,angle_output,tangle,z_offset, real_z_offset;
float now_theta, delta_theta, prev_theta, dx_raw, dy_raw, raw_x, raw_y, rot_x, rot_y, dx_rot, dy_rot, dx, dy;
double actual_x, actual_y;
float radnow[4], tacnow[4], tacprev[4], headingnow, delta_tac[4], dx_swerve, dy_swerve, x_swerve, y_swerve, real_dx_swerve, real_dy_swerve;
float Filtered_Angle;
float stepx, stepy, stepw, Vx, Vy, w, tVx, tVy,tVw, tw, full_stepx, full_stepy, full_stepw,xRvel, yRvel,wRvel;
R2CAN_Path          path;
Mov_Ave_t TF_L_MA;
Mov_Ave_t TF_R_MA;
Mov_Ave_t TF_F_MA;
Mov_Ave_t TF_B_MA;

void initialize(void){


//------------------- OUTPUT -------------------//
	// LEDs
	H7_gpio_PinInit(LED1_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(LED2_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(LED3_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(LED4_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(LED5_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(LED6_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);

	// DIG Ports
//	H7_gpio_PinInit(DIG1_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG2_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG3_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG4_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG5_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG6_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG7_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG8_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG9_PIN, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);

//------------------- INPUT -------------------//
	// Push Buttons
	H7_gpio_PinInit(PB1_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(PB2_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(PB3_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);

	// DIG Ports
	H7_gpio_PinInit(DIG1_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(DIG2_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(DIG3_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
	H7_gpio_PinInit(DIG4_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG5_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG6_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG7_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG8_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);
//	H7_gpio_PinInit(DIG9_PIN, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL);


//------------------- Timers -------------------//
/**
 * @brief Timers and PWM initialization
 * @warning TIM6 is used for RTOS, so it can not be used
 * @note Choose the t
 */

//** Initialize the structure **//
	H7_TIMx_init_strcut(&h7timer7, &htim7);
	H7_TIMx_init_strcut(&h7timer3, &htim3);
//	H7_TIMx_init_strcut(&h7timer12, &htim12);

//** PWM Configuration **//
//	H7_PWM_TIM_config(&h7timer3, 275, 20000);
//	H7_PWM_channel_config(&h7timer3, TIM_CHANNEL_3);

//** Timer Configuration **//
	H7_TIMx_init(&h7timer7, 275, 5000); 	// Basic timer for 5 ms operations
	H7_TIMx_init(&h7timer3, 275, 1000);		// Basic timer for 1 ms
	H7_TIMx_init(&h7timer12, 275, 5000); 	// Basic timer for 50 Hz, 20ms

//** Encoder Configuration **//
	H7_QEI_init(&QEI1, &htim1);		// X
	H7_QEI_init(&QEI2, &htim8);		// Y


//------------------- Communication Peripherals -------------------//

//----------- UART -----------//
/**
 * @brief 	UART initialization
 * @warning The ports name in H7 board first version are different from actual peripheral number
 * @note 	Interrupt is already enabled
 */
//** Initialize the structure **//
	H7_UARTx_init_struct(&h7uart5, &huart5); 	// UART1
	H7_UARTx_init_struct(&h7uart7, &huart7); 	// UART2
	H7_UARTx_init_struct(&h7uart4, &huart9);	// UART3
	H7_UARTx_init_struct(&h7uart9, &huart9); 	// UART4

//** Configure the port **//
	H7_UARTx_init(&h7uart5, 115200); 		// labeled UART1
	H7_UARTx_init(&h7uart7, 115200);	// tof		// labeled UART2
	H7_UARTx_init(&h7uart4, 115200);	// Bluebee 	// labeled UART3
	H7_UARTx_init(&h7uart9, 115200); // imu	// labeled UART4

//** Configure the port as DMA**//


//----------- I2C -----------//
/**
 * @brief 	I2C initialization
 * @warning Ports are labeled I1 and I2, which corresponds to I2C1 and I2C5
 * @note 	Interrupt is already enabled
 */
//** Initialize the structure **//
	H7_i2cx_init_struct(&h7i2c1, &hi2c1); 	// I1 for esp controller
	H7_i2cx_init_struct(&h7i2c5, &hi2c5);	// I2 for TOF

//** Configure the port **//
	H7_i2cx_init(&h7i2c1, 8, I2C_ADDRESSINGMODE_7BIT, CLK_SPEED_400KHz);
	H7_i2cx_init(&h7i2c5, 9, I2C_ADDRESSINGMODE_7BIT, CLK_SPEED_400KHz);

//** Configure the port as DMA**//
//	H7_i2cx_DMA_RX_init(&h7i2c1, DMA_CIRCULAR, 8, I2C_ADDRESSINGMODE_7BIT, CLK_SPEED_400KHz);


//----------- FDCAN -----------//
/**
 * @brief 	FDCAN initialization
 * @warning You have to define the ports that you are going to use in FDCAN.h
 * @warning Ports labeling on the board are different from actual peripheral number:
 * FDCAN1 --> CAN2
 * FDCAN2 --> CAN1
 * FDCAN3 --> CAN3
 * @note 	Interrupt is already enabled
 * @note 	You have to be careful with configuration parameters as they determine
 * the needed memory for the FDCAN, refer to the documentation FDCAN section
 */
#ifdef H7FDCAN1
	//** Initialize the structure **//
	if(H7_FDCANHandler_init(&h7fdcan1, &hfdcan1) != H7_PERIPH_OK){
		Error_Handler();
	}
	//** Configure the port **//
	if(H7_FDCAN_init(&h7fdcan1, 0, 0, 16, 0, 32, FDCAN_TX_FIFO_OPERATION, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_1Mbps, H7_FDCAN_8_BYTES) != H7_PERIPH_OK){
		Error_Handler();
	}
//** Initialize filters in case it is needed **//

//	if(H7_FDCAN_addFilterRange(&h7fdcan1, FDCAN_STANDARD_ID, 0x201, 0x208, FDCAN_FILTER_TO_RXFIFO0) != H7_FDCAN_FILTER_OK){
//		Error_Handler();
//	}
#endif

#ifdef H7FDCAN2
	//** Initialize the structure **//
	if(H7_FDCANHandler_init(&h7fdcan2, &hfdcan2) != H7_PERIPH_OK){
		Error_Handler();
	}
	//** Configure the port **//
	if(H7_FDCAN_init(&h7fdcan2, 0, 0, 16, 0, 32, FDCAN_TX_FIFO_OPERATION, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_1Mbps, H7_FDCAN_8_BYTES) != H7_PERIPH_OK){
		Error_Handler();
	}
//	if(H7_FDCAN_addFilterRange(&h7fdcan2, FDCAN_EXTENDED_ID, 0x201, 0x208, FDCAN_FILTER_TO_RXFIFO0) != H7_FDCAN_FILTER_OK){
//		Error_Handler();
//	}
#endif

#ifdef H7FDCAN3
	//** Initialize the structure **//
	if(H7_FDCANHandler_init(&h7fdcan3, &hfdcan3) != H7_PERIPH_OK){
		Error_Handler();
	}
	//** Configure the port **//
	if(H7_FDCAN_init(&h7fdcan3, 1, 1, 16, 0, 32, FDCAN_TX_FIFO_OPERATION, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_1Mbps, H7_FDCAN_8_BYTES) != H7_PERIPH_OK){
		Error_Handler();
	}
//	if(H7_FDCAN_addFilterRange(&h7fdcan3, FDCAN_EXTENDED_ID, 110, 116, FDCAN_FILTER_TO_RXFIFO1) != H7_FDCAN_FILTER_OK){
//		Error_Handler();
//	}
#endif


//------------------- Sensors -------------------//
//	IMU_Init(&IMU, &huart9);
	WTIMU63_Init(&IMU63,&huart9);

	PSxSlaveInit(&ps4, &hi2c1);
}


void CAN_PROCESS(PACKET_t packet_src) {

	switch (packet_src) {
	case VESC_PACKET:
		decode_VESC();
		if (vesc.error_flag) {
			vesc.error_flag = 0;
		}
		break;

	case ODRIVE_PACKET:

		break;

	case RNS_PACKET:
//		if (insData_receive[0] == 1) {
//			rns.RNS_data.common_instruction = insData_receive[1];
//			insData_receive[0] = 2;
//		}
//		if (insData_receive[0] == 17) {
//			if (buf2_flag == 1) {
//				rns.RNS_data.common_instruction = insData_receive[1];
//				rns.RNS_data.common_buffer[0].data = buf1_receive[0].data;
//				rns.RNS_data.common_buffer[1].data = buf1_receive[1].data;
//				rns.RNS_data.common_buffer[2].data = buf2_receive[0].data;
//				rns.RNS_data.common_buffer[3].data = buf2_receive[1].data;
//				insData_receive[0] = 3;
//			}
//		}

		break;

	case RBMS_PACKET:
		break;
	case CYBERGEAR_PACKET:

		break;
	default:
		break;
	}

}

#include "../../Actuators/VESC_FDCAN/vesc_interface.h"

static void bldc_send_packet(uint8_t controller_id, uint8_t *data, unsigned int len);
static int vescused = 1;
/*
 * Function Name		: VESCInit
 * Function Description : Initialize the bldc parameters
 * Function Remarks		: NONE
 * Function Arguments	: id1			VESC1 can id
 * 						  id2			VESC2 can id
 * 						  id3			VESC3 can id
 * 						  id4			VESC4 can id
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCInit(VESC1, VESC2, VESC3, VESC4, &vesc);
 */
void VESCInit(uint8_t id1, uint8_t id2, uint8_t id3, uint8_t id4, VESC_t* vesc){
	bldc_interface_init((void *)&bldc_send_packet);
//	bldc_interface_set_rx_value_selective_func(bldc_val_selective_received_cb);
	vesc->max_rpm = MCCONF_L_RPM_MAX;
	vesc->pole_pairs = MCCONF_SI_MOTOR_POLES;
	vesc->wheel_diameter = MCCONF_SI_WHEEL_DIAMETER;
	vesc->gear_ratio = MCCONF_SI_GEAR_RATIO;
	vesc->Instance1 = id1;
	vesc->Instance2 = id2;
	vesc->Instance3 = id3;
	vesc->Instance4 = id4;
	vesc->max_duty = MCCONF_L_MAX_DUTY;
	vesc->max_curr = MCCONF_L_CURRENT_MAX;

	//VOLTAGE_IN LIMIT
//	comm_can_conf_battery_cut(id1, Edit, 24.0, 23.5);
//	comm_can_conf_battery_cut(id2, Edit, 24.0, 23.5);
//	comm_can_conf_battery_cut(id3, Edit, 24.0, 23.5);
//	comm_can_conf_battery_cut(id4, Edit, 24.0, 23.5);

	//MOTOR_CURRENT
//	comm_can_conf_current_limits(id1, Edit, -20, 20);
//	comm_can_conf_current_limits(id2, Edit, -20, 20);
//	comm_can_conf_current_limits(id3, Edit, -20, 20);
//	comm_can_conf_current_limits(id4, Edit, -20, 20);

	//BATTERY_CURRENT
//	comm_can_conf_current_limits_in(id1, Edit, -0.1, 2);
//	comm_can_conf_current_limits_in(id2, Edit, -0.1, 2);
//	comm_can_conf_current_limits_in(id3, Edit, -0.1, 2);
//	comm_can_conf_current_limits_in(id4, Edit, -0.1, 2);

	vesc->protocol = 0;
	vesc->CAN = 1;
	vescused = 0;
}

/*
 * Function Name		: VESCUARTInit
 * Function Description : Initialize the bldc parameters
 * Function Remarks		: NONE
 * Function Arguments	: id1			VESC1 can id
 * 						  id2			VESC2 can id
 * 						  id3			VESC3 can id
 * 						  id4			VESC4 can id
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCUARTInit(VESC1, VESC2, VESC3, VESC4, &huart5, &vesc);
 */
void VESCUARTInit(UART_HandleTypeDef* huart1, UART_HandleTypeDef* huart2, UART_HandleTypeDef* huart3, UART_HandleTypeDef* huart4, VESC_t* vesc){
	bldc_interface_init((void *)&bldc_send_packet);
//	bldc_interface_set_rx_value_selective_func(bldc_val_selective_received_cb);
	vesc->pole_pairs = MCCONF_SI_MOTOR_POLES;
	vesc->wheel_diameter = MCCONF_SI_WHEEL_DIAMETER;
	vesc->gear_ratio = MCCONF_SI_GEAR_RATIO;

	vesc->max_rpm = MCCONF_L_RPM_MAX;
	vesc->max_duty = MCCONF_L_MAX_DUTY;
	vesc->max_curr = MCCONF_L_CURRENT_MAX;
	vesc->huartx4 = *huart1;
	vesc->huartx5 = *huart2;
	vesc->huartx7 = *huart3;
	vesc->huartx9 = *huart4;

	vesc->protocol = 0;
	vesc->UART = 1;
	vescused = 0;
}

/*
 * Function Name		: VESCVelocity
 * Function Description : Command the VESC to move with specified velocity without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: FLeftVel		speed of front left motor in meter per second
 * 						  FRightVel		speed of front right motor in meter per second
 * 						  BLeftVel 		speed of back left motor in meter per second
 * 						  BRightVel		speed of back right motor in meter per second
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCVelocity(1.0, 1.5 , 1.5 , 1.0, &vesc);
 */
void VESCVelocity(float FLeftVel, float FRightVel, float BLeftVel, float BRightVel, VESC_t* vesc) {

	float vel[4], rpm[4];
//	vel = ((Velocity * 60) / (M_PI * vesc->wheel_diameter)) * vesc->pole_pairs;
	vel[0] = ((FLeftVel * 60) / (M_PI * vesc->wheel_diameter)) * vesc->gear_ratio;
	rpm[0] = vel[0] * vesc->pole_pairs / 2;
	vel[1] = ((FRightVel * 60) / (M_PI * vesc->wheel_diameter)) * vesc->gear_ratio;
	rpm[1] = vel[1] * vesc->pole_pairs / 2;
	vel[2] = ((BLeftVel * 60) / (M_PI * vesc->wheel_diameter)) * vesc->gear_ratio;
	rpm[2] = vel[2] * vesc->pole_pairs / 2;
	vel[3] = ((BRightVel * 60) / (M_PI * vesc->wheel_diameter)) * vesc->gear_ratio;
	rpm[3] = vel[3] * vesc->pole_pairs / 2;

	VESCRPM(rpm[0], rpm[1], rpm[2], rpm[3], vesc);
}

/*
 * Function Name		: VESCRPM
 * Function Description : Command the VESC to move with specified ERPM without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: FLeftRPM		speed of front left motor in revolution per minute
 * 						  FRightRPM		speed of front right motor in revolution per minute
 * 						  BLeftRPM 		speed of back left motor in revolution per minute
 * 						  BRightRPM		speed of back right motor in revolution per minute
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCRPM(2000, 2000, 2000, 2000, &vesc);
 */
void VESCRPM(float FLeftRPM, float FRightRPM, float BLeftRPM, float BRightRPM, VESC_t* vesc) {
	if(vescused)
		return;

	chlim(FLeftRPM, vesc->max_rpm)
	chlim(FRightRPM, vesc->max_rpm)
	chlim(BLeftRPM, vesc->max_rpm)
	chlim(BRightRPM, vesc->max_rpm)

	if(vesc->stop){
		FLeftRPM = 0.0;
		FRightRPM = 0.0;
		BLeftRPM = 0.0;
		BRightRPM = 0.0;
	}

	if(vesc->CAN){
		comm_fdcan_set_rpm(vesc->Instance1, FLeftRPM);
		comm_fdcan_set_rpm(vesc->Instance2, FRightRPM);
		comm_fdcan_set_rpm(vesc->Instance3, BLeftRPM);
		comm_fdcan_set_rpm(vesc->Instance4, BRightRPM);
	}else if(vesc->UART){
		uart_bldc_interface_set_rpm(0.0);
//		uint8_t uart[10]; int32_t simplyPut=3;
//		uart[0] = 0x02;
//		uart[1] = 5;
//		uart[2] = COMM_SET_RPM;
//		buffer_append_int32(uart, FLeftRPM, &simplyPut);
//		unsigned short crc = crc16(&uart[2], 5);
//		uart[7] = (uint8_t)(crc >> 8);
//		uart[8] = (uint8_t)(crc & 0xFF);
//		uart[9] = 0x03;
//
//		HAL_UART_Transmit(&vesc->huartx1, (uint8_t *)uart, 10,10);
	}

	vesc->rpm_flag = 1;
}

/*
 * Function Name		: VESCPDC
 * Function Description : Command the VESC to move with specified duty cycle without any position control.
 * Function Remarks		: The range of duty cycle : -0.95 to 0.95
 * Function Arguments	: FLeftPDC		duty cycle of front left motor
 * 						  FRightPDC		duty cycle of front right motor
 * 						  BLeftPDC 		duty cycle of back left motor
 * 						  BRightPDC		duty cycle of back right motor
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCPDC(0.5, 0.5, 0.5, 0.5, &vesc);
 */
void VESCPDC(float FLeftPDC, float FRightPDC, float BLeftPDC, float BRightPDC, VESC_t* vesc) {
	if(vescused)
		return;

	if(vesc->stop == 1){
		FLeftPDC = 0.0;
		FRightPDC = 0.0;
		BLeftPDC = 0.0;
		BRightPDC = 0.0;
	}

	chlim(FLeftPDC, vesc->max_duty)
	chlim(FRightPDC, vesc->max_duty)
	chlim(BLeftPDC, vesc->max_duty)
	chlim(BRightPDC, vesc->max_duty)

	comm_fdcan_set_duty(vesc->Instance1, FLeftPDC);
	comm_fdcan_set_duty(vesc->Instance2, FRightPDC);
	comm_fdcan_set_duty(vesc->Instance3, BLeftPDC);
	comm_fdcan_set_duty(vesc->Instance4, BRightPDC);

//	uint8_t uart[10]; int32_t simplyPut=3;
//	uart[0] = 0x02;
//	uart[1] = 5;
//	uart[2] = COMM_SET_DUTY;
//	buffer_append_int32(uart, (int32_t)(duty * 100000.0), &simplyPut);
//	unsigned short crc = crc16(&uart[2], 5);
//	uart[7] = (uint8_t)(crc >> 8);
//	uart[8] = (uint8_t)(crc & 0xFF);
//	uart[9] = 0x03;
//
//	HAL_UART_Transmit(&vesc->huartx1, (uint8_t *)uart, 10,10);

	vesc->pdc_flag = 1;
}

/*
 * Function Name		: VESCPOS
 * Function Description : Command the VESC to move with specified position angle by position control.
 * Function Remarks		: The range of duty cycle : 0.0 to 360.0
 * 						  MUST use BLDC with encoder
 * Function Arguments	: FLeftPOS		angular position of front left motor
 * 						  FRightPOS		angular position of front right motor
 * 						  BLeftPOS 		angular position of back left motor
 * 						  BRightPOS		angular position of back right motor
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCPOS(33.3, 66.6, 99.9, 123.4, &vesc);
 */
void VESCPOS(float FLeftPOS, float FRightPOS, float BLeftPOS, float BRightPOS, VESC_t* vesc){
	if(vescused)
		return;

	Llim(FLeftPOS, 0.0);
	Llim(FRightPOS, 0.0);
	Llim(BLeftPOS, 0.0);
	Llim(BRightPOS, 0.0);
	Ulim(FLeftPOS, 360.0);
	Ulim(FRightPOS, 360.0);
	Ulim(BLeftPOS, 360.0);
	Ulim(BRightPOS, 360.0);

	comm_fdcan_set_pos(vesc->Instance1, FLeftPOS);
	comm_fdcan_set_pos(vesc->Instance2, FRightPOS);
	comm_fdcan_set_pos(vesc->Instance3, BLeftPOS);
	comm_fdcan_set_pos(vesc->Instance4, BRightPOS);
}

/*
 * Function Name		: VESCCurr
 * Function Description : Command the VESC to move with specified current without position control.
 * Function Remarks		: This control is not very stable
 * Function Arguments	: FLeftCurr		Current to front left motor
 * 						  FRightCurr	Current to front right motor
 * 						  BLeftCurr		Current to back left motor
 * 						  BRightCurr	Current to back right motor
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCCurr(3.3, 6.6, 1.1, 1.234, &vesc);
 */
void VESCCurr(float FLeftCurr, float BLeftCurr, float FRightCurr, float BRightCurr, VESC_t* vesc) {
	if(vescused)
		return;

	if(vesc->stop){
		FLeftCurr = 0.0;
		FRightCurr = 0.0;
		BLeftCurr = 0.0;
		BRightCurr = 0.0;
	}

	chlim(FLeftCurr, vesc->max_curr)
	chlim(FRightCurr, vesc->max_curr)
	chlim(BLeftCurr, vesc->max_curr)
	chlim(BRightCurr, vesc->max_curr)

	comm_fdcan_set_current(vesc->Instance1, FLeftCurr);
	comm_fdcan_set_current(vesc->Instance2, BLeftCurr);
	comm_fdcan_set_current(vesc->Instance3, FRightCurr);
	comm_fdcan_set_current(vesc->Instance4, BRightCurr);

	vesc->current_flag = 1;
}

/*
 * Function Name		: VESCStop
 * Function Description : Command the VESC to stop
 * Function Remarks		: NONE
 * Function Arguments	: vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCStop(&vesc);
 */
void VESCStop(VESC_t* vesc) {
	if(vescused)
		return;

	if(vesc->CAN){
		if(vesc->pdc_flag) {
			comm_fdcan_set_duty(vesc->Instance1, 0.0);
			comm_fdcan_set_duty(vesc->Instance2, 0.0);
			comm_fdcan_set_duty(vesc->Instance3, 0.0);
			comm_fdcan_set_duty(vesc->Instance4, 0.0);
		}else if(vesc->rpm_flag) {
			comm_fdcan_set_rpm(vesc->Instance1, 0.0);
			comm_fdcan_set_rpm(vesc->Instance2, 0.0);
			comm_fdcan_set_rpm(vesc->Instance3, 0.0);
			comm_fdcan_set_rpm(vesc->Instance4, 0.0);
		}else if(vesc->current_flag){
			comm_fdcan_set_current(vesc->Instance1, 0.0);
			comm_fdcan_set_current(vesc->Instance2, 0.0);
			comm_fdcan_set_current(vesc->Instance3, 0.0);
			comm_fdcan_set_current(vesc->Instance4, 0.0);
		}
	}else if(vesc->UART){
//		if(vesc->pdc_flag) {
//			comm_can_set_duty(vesc->Instance1, 0.0);
//			comm_can_set_duty(vesc->Instance2, 0.0);
//			comm_can_set_duty(vesc->Instance3, 0.0);
//			comm_can_set_duty(vesc->Instance4, 0.0);
//		}else if(vesc->rpm_flag) {
//			comm_can_set_rpm(vesc->Instance1, 0.0);
//			comm_can_set_rpm(vesc->Instance2, 0.0);
//			comm_can_set_rpm(vesc->Instance3, 0.0);
//			comm_can_set_rpm(vesc->Instance4, 0.0);
//		}else if(vesc->current_flag){
//			comm_can_set_current(vesc->Instance1, 0.0);
//			comm_can_set_current(vesc->Instance2, 0.0);
//			comm_can_set_current(vesc->Instance3, 0.0);
//			comm_can_set_current(vesc->Instance4, 0.0);
//		}
		if(vesc->rpm_flag){
			uart_bldc_interface_set_rpm(0.0);
		}
	}

	vesc->flags = 0;
}

/*
 * Function Name		: VESCReleaseMotor
 * Function Description : Command the VESC to stop and swap freely
 * Function Remarks		: Edit VESC board
 * Function Arguments	: vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCReleaseMotor(&vesc);
 */
void VESCReleaseMotor(VESC_t* vesc){
	if(vescused)
		return;
#ifdef VESCedit
	comm_can_release_motor(vesc->Instance1);
	comm_can_release_motor(vesc->Instance2);
	comm_can_release_motor(vesc->Instance3);
	comm_can_release_motor(vesc->Instance4);
#else
	comm_fdcan_set_current(vesc->Instance1, MCCONF_CC_MIN_CURRENT / 2.0);
	comm_fdcan_set_current(vesc->Instance2, MCCONF_CC_MIN_CURRENT / 2.0);
	comm_fdcan_set_current(vesc->Instance3, MCCONF_CC_MIN_CURRENT / 2.0);
	comm_fdcan_set_current(vesc->Instance4, MCCONF_CC_MIN_CURRENT / 2.0);
#endif
}

/*
 * Function Name		: VESCHandBrake
 * Function Description : Command the VESC to handbrake
 * Function Remarks		: NONE
 * Function Arguments	: HBrakeCurr	Handbrake current
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCHandBrake(3.0, &vesc);
 */
void VESCHandBrake(float HBrakeCurr, VESC_t* vesc){
	if(vescused)
		return;

	comm_fdcan_set_handbrake(vesc->Instance1, HBrakeCurr);
	comm_fdcan_set_handbrake(vesc->Instance2, HBrakeCurr);
	comm_fdcan_set_handbrake(vesc->Instance3, HBrakeCurr);
	comm_fdcan_set_handbrake(vesc->Instance4, HBrakeCurr);
}

/*
 * Function Name		: VESCCurrBrake
 * Function Description : Command the VESC to brake
 * Function Remarks		: NONE
 * Function Arguments	: BrakeCurr		Brake current
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCHCurrBrake(3.0, &vesc);
 */
void VESCCurrBrake(float BrakeCurr, VESC_t* vesc){
	if(vescused)
		return;

	comm_fdcan_set_current_brake(vesc->Instance1, BrakeCurr);
	comm_fdcan_set_current_brake(vesc->Instance2, BrakeCurr);
	comm_fdcan_set_current_brake(vesc->Instance3, BrakeCurr);
	comm_fdcan_set_current_brake(vesc->Instance4, BrakeCurr);
}


void VESCFullBrake(VESC_t* vesc){
	comm_fdcan_brake(vesc->Instance1);
	comm_fdcan_brake(vesc->Instance2);
	comm_fdcan_brake(vesc->Instance3);
	comm_fdcan_brake(vesc->Instance4);


}

/*	Private Function	*/
void bldc_send_packet(uint8_t controller_id, uint8_t *data, unsigned int len) {
		comm_fdcan_send_buffer(controller_id, data, len, 0);
//		HAL_UART_Transmit(&(vesc.huartx), (uint8_t *)data, len, 100);
}

/*
 * Function Name		: VESCgetSpeed
 * Function Description : Calculate BLDC speed in metre per second
 * Function Remarks		: call VESCDATA before used
 * Function Arguments	: vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCgetSpeed(&vesc);
 */
void VESCGetSpeed(VESC_t* vesc) {
	if(vescused)
		return;

	const float FLeftRPM = infolist[0].rpm / (vesc->pole_pairs / 2.0);
	const float FRightRPM = infolist[1].rpm / (vesc->pole_pairs / 2.0);
	const float BLeftRPM = infolist[2].rpm / (vesc->pole_pairs / 2.0);
	const float BRightRPM = infolist[3].rpm / (vesc->pole_pairs / 2.0);
	infolist[0].speed = (FLeftRPM / 60.0) * vesc->wheel_diameter * M_PI / vesc->gear_ratio;
	infolist[1].speed = (FRightRPM / 60.0) * vesc->wheel_diameter * M_PI / vesc->gear_ratio;
	infolist[2].speed = (BLeftRPM / 60.0) * vesc->wheel_diameter * M_PI / vesc->gear_ratio;
	infolist[3].speed = (BRightRPM / 60.0) * vesc->wheel_diameter * M_PI / vesc->gear_ratio;
}

/*
 * Function Name		: VESCgetDist
 * Function Description : Calculate total distance traveled in metre per second
 * Function Remarks		: call VESCDATA before used
 * Function Arguments	: vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCgetDist(&vesc);
 */
void VESCGetDist(VESC_t* vesc) {
	if(vescused)
		return;

	const float tacho_scale = (vesc->wheel_diameter * M_PI) / (3.0 * vesc->pole_pairs * vesc->gear_ratio);
	infolist[0].dist = infolist[0].tacho_value * tacho_scale;
	infolist[1].dist = infolist[1].tacho_value * tacho_scale;
	infolist[2].dist = infolist[2].tacho_value * tacho_scale;
	infolist[3].dist = infolist[3].tacho_value * tacho_scale;
}

#ifdef VESCedit

/*
 * Function Name		: VESCEnquire
 * Function Description : Enquire the VESC boards and the motor parameters.  Call once only
 * Function Remarks		: Must upload our VESC code to VESC before use
 * Function Arguments	: status		CAN_status
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCEnquire(CAN_STATUS_1 | CAN_STATUS_4 | CAN_STATUS_5, &vesc);
 */
void VESCEnquire(uint32_t status, VESC_t* vesc) {
	if(vescused)
		return;
	comm_can_packet_status(vesc->Instance1, status);
	comm_can_packet_status(vesc->Instance2, status);
	comm_can_packet_status(vesc->Instance3, status);
	comm_can_packet_status(vesc->Instance4, status);
}

/*
 * Function Name		: VESCPOS_offset
 * Function Description : Set position offset value specific VESC
 * Function Remarks		: NONE
 * Function Arguments	: arg		offset angle
 * 						  flash		Write in flash of VESC board. Edit do nothing on flash
 * 						  id		VESC can id
 * Function Return		: None
 * Function Example		: VESCPOS_offset(33.3, Edit, VESC1,&vesc);
 */
void VESCSetPosOffset(float arg, unsigned char flash, uint16_t id){
	if(vescused)
		return;

	comm_can_set_pos_offset(id, arg, flash);
}

/*
 * Function Name		: VESCsetDist
 * Function Description : Set distance offset value of specific VESC
 * Function Remarks		: NONE
 * Function Arguments	: arg		distance offset
 * 						  id		VESC can id
 * Function Return		: None
 * Function Example		: VESCsetDist(33.3, VESC1);
 */
void VESCSetDist(float dist, uint16_t id, VESC_t* vesc){
	if(vescused)
		return;

	const float tacho_scale = (vesc->wheel_diameter * M_PI) / (3.0 * vesc->pole_pairs * vesc->gear_ratio);
	float techometer = dist / tacho_scale;
	comm_can_set_tachometer(id, techometer);
}
#endif

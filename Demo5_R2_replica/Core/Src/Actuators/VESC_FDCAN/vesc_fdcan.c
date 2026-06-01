/*
 * vesc_can.c Previously
 *  Created on: Dec 23, 2020
 *      Author: root
 *
 * vesc_fdcan.c Currently
 *  modified on: Dec 23, 2020
 *      Author: Mohammed Abdulalem
 *      note: The modification just to satisfy FD
 */

#include "../../Actuators/VESC_FDCAN/vesc_fdcan.h"

#include "../../adapter.h"

static unsigned int rx_buffer_last_id;
static Vescmsg rx_frames[50];
static int rx_frame_read = 0;
static int rx_frame_write = 0;


// Global variables
uint8_t rx_buffer[RX_BUFFER_SIZE];
io_board_adc_values io_board_adc_1_4[10];
io_board_adc_values io_board_adc_5_8[10];

char vescerror [100];

volatile vesc_enq infolist[VESCPOSLAST - VESC1];

VESC_t vesc;
volatile float moved_pos[VESCPOSLAST - VESC1];
volatile uint8_t pos_overflow[VESCPOSLAST - VESC1];

Vescmsg vescmsg;
/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * 0: Packet goes to commands_process_packet of receiver
 * 1: Packet goes to commands_send_packet of receiver --> can use to send to other board?
 * 2: Packet goes to commands_process and send function is set to null
 *    so that no reply is sent back.
 */

void comm_fdcan_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = (uint8_t)RNS_TO_VESC;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
#if defined VESC_FDCAN1
		FDCAN_TxMsgEID(&hfdcan1, controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
#elif defined VESC_FDCAN2
		FDCAN_TxMsgEID(&hfdcan2, controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
#elif defined VESC_FDCAN3
		FDCAN_TxMsgEID(&hfdcan3, controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
#endif
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}
#if defined VESC_FDCAN1
			FDCAN_TxMsgEID(&hfdcan1, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
#elif defined VESC_FDCAN2
			FDCAN_TxMsgEID(&hfdcan2, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
#elif defined VESC_FDCAN3
			FDCAN_TxMsgEID(&hfdcan3, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
#endif

		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

#if defined VESC_FDCAN1
			FDCAN_TxMsgEID(&hfdcan1, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
#elif defined VESC_FDCAN2
			FDCAN_TxMsgEID(&hfdcan2, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
#elif defined VESC_FDCAN3
			FDCAN_TxMsgEID(&hfdcan3, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
#endif
		}

		uint32_t ind = 0;
		send_buffer[ind++] = (uint8_t)RNS_TO_VESC;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

#if defined VESC_FDCAN1
		FDCAN_TxMsgEID(&hfdcan1, controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
#elif defined VESC_FDCAN2
		FDCAN_TxMsgEID(&hfdcan2, controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
#elif defined VESC_FDCAN3
		FDCAN_TxMsgEID(&hfdcan3, controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
#endif
	}
}

void comm_fdcan_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 1e5), &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
#endif
}

void comm_fdcan_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#endif
}

/**
 * the current will run at max motor current, ignore the urrent value set.
 * The time has no significant changes
 *
 * Set current off delay in second. Prevent the current controller from switching off modulation
 * for target currents < cc_min_current for this amount of time.
 */
void comm_fdcan_set_current_off_delay(uint8_t controller_id, float current, float off_delay) {
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#endif
}

void comm_fdcan_set_current_brake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
#endif
}

void comm_fdcan_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
#endif
}

void comm_fdcan_move_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0, vesc_index = 0;
	uint8_t buffer[4];

	vesc_index = controller_id - VESC1;

	moved_pos[vesc_index] = infolist[vesc_index].tol_angle + pos;

	//	if(moved_pos[vesc_index] >= 360.0){
	//		moved_pos[vesc_index] -= 150.0;
	//		pos_overflow[vesc_index] = 1;
	//	}else if(moved_pos[vesc_index] <= -360.0){
	//		moved_pos[vesc_index] += 150.0;
	//		pos_overflow[vesc_index] = 1;
	//	}

	ch360(moved_pos[vesc_index]);

	buffer_append_int32(buffer, (int32_t)(moved_pos[vesc_index] * 1000000.0), &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#endif

	//	while(fabsf(moved_pos[vesc_index]) >= 390.0)
	//		ch360(moved_pos[vesc_index]);
	//	ch360(moved_pos[vesc_index]);
	//	comm_can_set_pos(controller_id, moved_pos[vesc_index]);
}

void comm_fdcan_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0, vesc_index = 0;
	uint8_t buffer[4];

	vesc_index = controller_id - VESC1;
	moved_pos[vesc_index] = infolist[vesc_index].tol_angle + pos;
	ch360(pos);

	buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#endif
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [-1.0 1.0]
 */
void comm_fdcan_set_current_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
#endif
}

/**
 * Set brake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [0.0 1.0]
 */
void comm_fdcan_set_current_brake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
#endif
}

/**
 * Set handbrake current.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The handbrake current value
 */
void comm_fdcan_set_handbrake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
#endif
}

/**
 * Set handbrake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The relative handbrake current value, range [0.0 1.0]
 */
void comm_fdcan_set_handbrake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
#endif
}

void comm_fdcan_packet_ADC14(uint8_t controller_id) {
	int32_t send_index = 0;
	uint8_t buffer = 0;
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_IO_BOARD_ADC_1_TO_4 << 8), (uint8_t *)&buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_IO_BOARD_ADC_1_TO_4 << 8), (uint8_t *)&buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_IO_BOARD_ADC_1_TO_4 << 8), (uint8_t *)&buffer, send_index);
#endif
}
void comm_fdcan_brake(uint8_t controller_id) {
	#if defined VESC_FDCAN1
		FDCAN_TxMsgEID(&hfdcan1, controller_id |
				((uint32_t)CAN_PACKET_BRAKE << 8), 0, 0);
	#elif defined VESC_FDCAN2
		FDCAN_TxMsgEID(&hfdcan2, controller_id |
					((uint32_t)CAN_PACKET_BRAKE << 8), 0, 0);
#elif defined VESC_FDCAN3
		FDCAN_TxMsgEID(&hfdcan3, controller_id |
					((uint32_t)CAN_PACKET_BRAKE << 8), 0, 0);
	#endif
	comm_fdcan_set_duty(controller_id, 0.0);
}

void comm_fdcan_packet_ADC58(uint8_t controller_id) {
	int32_t send_index = 0;
	uint8_t buffer = 0;
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_IO_BOARD_ADC_5_TO_8 << 8), (uint8_t *)&buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_IO_BOARD_ADC_5_TO_8 << 8), (uint8_t *)&buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_IO_BOARD_ADC_5_TO_8 << 8), (uint8_t *)&buffer, send_index);
#endif
}

#ifdef VESCedit
/**
 * get relative can status
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param status
 * The relative can status
 *
 * example
 * comm_can_packet_status(VESCPOS1, CAN_STATUS_1 | CAN_STATUS_4 | CAN_STATUS_5);
 */
void comm_can_packet_status(uint8_t controller_id, uint32_t status) {
	int32_t send_index = 0;
	uint8_t buffer[4];

	buffer_append_int32(buffer, status, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_GET_INFO << 8), (uint8_t *)&buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_GET_INFO << 8), (uint8_t *)&buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_GET_INFO << 8), (uint8_t *)&buffer, send_index);
#endif
}

void comm_can_set_pos_offset(uint8_t controller_id, float pos, unsigned char flash) {
	int32_t send_index = 0;
	uint8_t buffer[5];
	buffer_append_int32(buffer, (int32_t)(pos * 1e4), &send_index);
	buffer[send_index++] = flash;
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_UPDATE_PID_POS_OFFSET << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_UPDATE_PID_POS_OFFSET << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_UPDATE_PID_POS_OFFSET << 8), buffer, send_index);
#endif
}

void comm_can_set_tachometer(uint8_t controller_id, float techometer) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(techometer), &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_TACHOMETER << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_TACHOMETER << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_SET_TACHOMETER << 8), buffer, send_index);
#endif
}


void comm_can_release_motor(uint8_t controller_id) {
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)CAN_PACKET_RELEASE_MOTOR << 8), 0, 0);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_RELEASE_MOTOR << 8), 0, 0);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)CAN_PACKET_RELEASE_MOTOR << 8), 0, 0);
#endif
}
#endif

/**
 * Update current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param flash
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum motor current (negative value).
 *
 * @param max
 * Maximum motor current.
 */
void comm_fdcan_conf_current_limits(uint8_t controller_id, unsigned char flash, float min, float max) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, min, 1e3, &send_index);
	buffer_append_float32(buffer, max, 1e3, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS :
					CAN_PACKET_CONF_CURRENT_LIMITS) << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS :
					CAN_PACKET_CONF_CURRENT_LIMITS) << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS :
					CAN_PACKET_CONF_CURRENT_LIMITS) << 8), buffer, send_index);
#endif
}

/**
 * Update input current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param flash
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum battery current (negative value).
 *
 * @param max
 * Maximum battery current.
 */
void comm_fdcan_conf_current_limits_in(uint8_t controller_id, unsigned char flash, float min, float max) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, min, 1e3, &send_index);
	buffer_append_float32(buffer, max, 1e3, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN :
					CAN_PACKET_CONF_CURRENT_LIMITS_IN) << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN :
					CAN_PACKET_CONF_CURRENT_LIMITS_IN) << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN :
					CAN_PACKET_CONF_CURRENT_LIMITS_IN) << 8), buffer, send_index);
#endif
}

/**
 * Update input voltage under voltage limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param flash
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param start
 * Battery Voltage Cutoff Start
 *
 * @param end
 * Battery Voltage Cutoff End
 */
void comm_fdcan_conf_battery_cut(uint8_t controller_id, unsigned char flash, float start, float end) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, start, 1e3, &send_index);
	buffer_append_float32(buffer, end, 1e3, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_BATTERY_CUT :
					CAN_PACKET_CONF_BATTERY_CUT) << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_BATTERY_CUT :
					CAN_PACKET_CONF_BATTERY_CUT) << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_BATTERY_CUT :
					CAN_PACKET_CONF_BATTERY_CUT) << 8), buffer, send_index);
#endif
}

/**
 * Update FOC ERPM settings on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param foc_openloop_rpm
 * Run in openloop below this ERPM in sensorless mode.
 *
 * @param foc_sl_erpm
 * Use sensors below this ERPM in sensored mode.
 */
void comm_fdcan_conf_foc_erpms(uint8_t controller_id, unsigned char flash, float foc_openloop_rpm, float foc_sl_erpm) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, foc_openloop_rpm, 1e3, &send_index);
	buffer_append_float32(buffer, foc_sl_erpm, 1e3, &send_index);
#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_FOC_ERPMS :
					CAN_PACKET_CONF_FOC_ERPMS) << 8), buffer, send_index);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_FOC_ERPMS :
					CAN_PACKET_CONF_FOC_ERPMS) << 8), buffer, send_index);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id |
			((uint32_t)(flash ? CAN_PACKET_CONF_STORE_FOC_ERPMS :
					CAN_PACKET_CONF_FOC_ERPMS) << 8), buffer, send_index);
#endif
}

void comm_fdcan_set_pos_pid(uint8_t controller_id, float pos_kp, float pos_ki, float pos_kd) {
	int32_t send_index = 0;
	uint8_t buffer[12];
	buffer_append_int32(buffer, (int32_t)(pos_kp * 1e3), &send_index);
	buffer_append_int32(buffer, (int32_t)(pos_ki * 1e3), &send_index);
	buffer_append_int32(buffer, (int32_t)(pos_kd * 1e5), &send_index);

#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KP << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan1, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KI << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan1, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KD << 8), buffer, send_index / 3);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KP << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan2, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KI << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan2, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KD << 8), buffer, send_index / 3);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KP << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan3, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KI << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan3, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KD << 8), buffer, send_index / 3);
#endif
}

void comm_fdcan_set_rpm_pid(uint8_t controller_id, float rpm_kp, float rpm_ki, float rpm_kd) {
	int32_t send_index = 0;
	uint8_t buffer[12];
	buffer_append_int32(buffer, (int32_t)(rpm_kp * 1e3), &send_index);
	buffer_append_int32(buffer, (int32_t)(rpm_ki * 1e3), &send_index);
	buffer_append_int32(buffer, (int32_t)(rpm_kd * 1e5), &send_index);

#if defined VESC_FDCAN1
	FDCAN_TxMsgEID(&hfdcan1, controller_id | ((uint32_t)CAN_PACKET_SET_RPM_KP << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan1, controller_id | ((uint32_t)CAN_PACKET_SET_RPM_KI << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan1, controller_id | ((uint32_t)CAN_PACKET_SET_RPM_KD << 8), buffer, send_index / 3);
#elif defined VESC_FDCAN2
	FDCAN_TxMsgEID(&hfdcan2, controller_id | ((uint32_t)CAN_PACKET_SET_RPM_KP << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan2, controller_id | ((uint32_t)CAN_PACKET_SET_RPM_KI << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan2, controller_id | ((uint32_t)CAN_PACKET_SET_RPM_KD << 8), buffer, send_index / 3);
#elif defined VESC_FDCAN3
	FDCAN_TxMsgEID(&hfdcan3, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KP << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan3, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KI << 8), buffer, send_index / 3);
	FDCAN_TxMsgEID(&hfdcan3, controller_id | ((uint32_t)CAN_PACKET_SET_POS_KD << 8), buffer, send_index / 3);
#endif
}

void decode_VESC(void){
	int32_t ind = 0;
	uint8_t vesc_index = 0;
	unsigned int rxbuf_len = 0;
	unsigned int rxbuf_ind;
	uint8_t crc_low;
	uint8_t crc_high;
	uint8_t commands_send;
	int32_t indexbuf = 0;
//	char test[100];

	//ori
	//	Vescmsg *rxmsg_tmp;
	//	while ((rxmsg_tmp = get_rx_frame()) != 0) {
	//		Vescmsg rxmsg = *rxmsg_tmp;
	//
	//		if (rxmsg.Rxmsg.IDE == CAN_ID_EXT) {
	//			uint8_t id = rxmsg.Rxmsg.ExtId & 0xFF;
	//			CAN_PACKET_ID cmd = rxmsg.Rxmsg.ExtId >> 8;
	//
	//			if (id == 255 || id == RNS_TO_VESC) {
	//				switch (cmd) {

	uint8_t id = vescmsg.Rxmsg.Identifier & 0xFF;
	CAN_PACKET_ID cmd = vescmsg.Rxmsg.Identifier >> 8;
//	sprintf(test,"id : %d \n",id);
//	HAL_UART_Transmit(&huart2, test, strlen(test), 100);
	vesc_index = id - VESC1;

//	if (vesc_index == 0){
//		led5 = !led5;
//	}
//	if (vesc_index == 1){
//		led6 = !led6;
//	}
//	if (vesc_index == 2){
//		led7 = !led7;
//	}
	if(vesc_index < 0 || vesc_index > VESCPOSLAST - VESC1){	//out of id range
		vesc.error_flag = 1;
		sprintf(vescerror, "VESC id (%d) is not in range", (int)id);
		return;
	}else
		infolist[vesc_index].CANid = id;

	switch (cmd) {
	case CAN_PACKET_FILL_RX_BUFFER:
		memcpy(rx_buffer + vescmsg.Data[0], vescmsg.Data + 1, vescmsg.Rxmsg.DataLength - 1);
		break;

	case CAN_PACKET_FILL_RX_BUFFER_LONG:
		rxbuf_ind = (unsigned int)vescmsg.Data[0] << 8;
		rxbuf_ind |= vescmsg.Data[1];
		if (rxbuf_ind < RX_BUFFER_SIZE) {
			memcpy(rx_buffer + rxbuf_ind, vescmsg.Data + 2, vescmsg.Rxmsg.DataLength - 2);
		}
		break;

	case CAN_PACKET_PROCESS_RX_BUFFER:
		ind = 0;
		rx_buffer_last_id = vescmsg.Data[ind++];
		commands_send = vescmsg.Data[ind++];
		rxbuf_len = (unsigned int)vescmsg.Data[ind++] << 8;
		rxbuf_len |= (unsigned int)vescmsg.Data[ind++];

		if (rxbuf_len > RX_BUFFER_SIZE) {
			break;
		}

		crc_high = vescmsg.Data[ind++];
		crc_low = vescmsg.Data[ind++];

		if (crc16(rx_buffer, rxbuf_len)
				== ((unsigned short) crc_high << 8
						| (unsigned short) crc_low)) {
			if(commands_send==1)
				bldc_interface_process_packet(rx_buffer, rxbuf_len);
		}
		break;

	case CAN_PACKET_PROCESS_SHORT_BUFFER:
		ind = 0;
		rx_buffer_last_id = vescmsg.Data[ind++];
		commands_send = vescmsg.Data[ind++];

		if(commands_send==1)
			bldc_interface_process_packet(rx_buffer, rxbuf_len);
		break;

	case CAN_PACKET_STATUS:
		infolist[vesc_index].rpm = buffer_get_float32((uint8_t*)&vescmsg.Data, 1.0, &indexbuf);
		infolist[vesc_index].current = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		infolist[vesc_index].duty_cycle = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		break;
	case CAN_PACKET_STATUS_2:
		infolist[vesc_index].amp_hours = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		infolist[vesc_index].amp_hours_charged = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		break;
	case CAN_PACKET_STATUS_3:
		infolist[vesc_index].watt_hours = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		infolist[vesc_index].watt_hours_charged = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		break;
	case CAN_PACKET_STATUS_4:
		infolist[vesc_index].temp_fet = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		infolist[vesc_index].temp_motor = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		infolist[vesc_index].current_in = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		infolist[vesc_index].pos = buffer_get_float16((uint8_t*)&vescmsg.Data, 50, &indexbuf);
		break;
	case CAN_PACKET_STATUS_5:
		infolist[vesc_index].tacho_value = buffer_get_float32((uint8_t*)&vescmsg.Data, 1.0, &indexbuf) / (MCCONF_SI_MOTOR_POLES * 3);
		infolist[vesc_index].v_in = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &indexbuf);
		infolist[vesc_index].Fault = buffer_get_int16((uint8_t*)&vescmsg.Data, &indexbuf);
		break;
	case CAN_PACKET_STATUS_6:
		infolist[vesc_index].adc_1 = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		infolist[vesc_index].adc_2 = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		infolist[vesc_index].adc_3 = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		infolist[vesc_index].ppm = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		break;
	case CAN_PACKET_STATUS_7:
		infolist[vesc_index].speed = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e5, &indexbuf);
		infolist[vesc_index].dist = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e5, &indexbuf);
		break;
	case CAN_PACKET_STATUS_8:
		infolist[vesc_index].Id = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		infolist[vesc_index].Iq = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e2, &indexbuf);
		break;
	case CAN_PACKET_STATUS_9:
		infolist[vesc_index].Vd = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		infolist[vesc_index].Vq = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		break;
	case CAN_PACKET_STATUS_10:
		infolist[vesc_index].KV = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf) / (MCCONF_SI_MOTOR_POLES / 2);
		infolist[vesc_index].testdata = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		break;
	case CAN_PACKET_POLL_ROTOR_POS:
		infolist[vesc_index].enc_pos = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e5, &indexbuf);
		break;
	case CAN_PACKET_STATUS_11:
		infolist[vesc_index].rpm_kp = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		infolist[vesc_index].rpm_ki = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		break;
	case CAN_PACKET_STATUS_12:
		infolist[vesc_index].rpm_kd = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e5, &indexbuf);
		infolist[vesc_index].rpm_kdfilter = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e5, &indexbuf);
		break;
	case CAN_PACKET_STATUS_13:
		infolist[vesc_index].pos_kp = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		infolist[vesc_index].pos_ki = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e3, &indexbuf);
		break;
	case CAN_PACKET_STATUS_14:
		infolist[vesc_index].pos_kd = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e5, &indexbuf);
		infolist[vesc_index].pos_kdfilter = buffer_get_float32((uint8_t*)&vescmsg.Data, 1e5, &indexbuf);
		break;

	case CAN_PACKET_IO_BOARD_ADC_1_TO_4:
		//adc voltage
		for (int i = 0;i < 10;i++) {
			io_board_adc_values *msg = &io_board_adc_1_4[i];
			if (msg->id == id || msg->id == -1) {
				ind = 0;
				msg->id = id;
				//							msg->rx_time = HAL_GetTick();
				ind = 0;
				int j = 0;
				while (ind < rxbuf_len) {
					msg->adc_voltages[j++] = buffer_get_float16(vescmsg.Data, 1e2, &ind);
				}
				break;
			}
		}
		break;

	case CAN_PACKET_IO_BOARD_ADC_5_TO_8:
		//adc voltage
		for (int i = 0;i < 10;i++) {
			io_board_adc_values *msg = &io_board_adc_5_8[i];
			if (msg->id == id || msg->id == -1) {
				ind = 0;
				msg->id = id;
				//							msg->rx_time = HAL_GetTick();
				ind = 0;
				int j = 0;
				while (ind < rxbuf_len) {
					msg->adc_voltages[j++] = buffer_get_float16(vescmsg.Data, 1e2, &ind);
				}
				break;
			}
		}
		break;
	default:
		break;
	}
//	if(infolist[vesc_index].Fault){
//		vesc.error_flag = 1;
//		sprintf(vescerror, "VESC id: %d, ", (int)infolist[vesc_index].CANid);
//		strcat(vescerror, mc_interface_fault_to_string((mc_fault_code)infolist[vesc_index].Fault));
//		strcat(vescerror, "\n");
//	}
//	infolist[vesc_index].tol_angle = infolist[vesc_index].tacho_value * 360.0;
}

const char* mc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE";
	case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE";
	case FAULT_CODE_MCU_UNDER_VOLTAGE: return "FAULT_CODE_MCU_UNDER_VOLTAGE";
	case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET: return "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET";
	case FAULT_CODE_ENCODER_SPI: return "FAULT_CODE_ENCODER_SPI";
	case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
	case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
	case FAULT_CODE_FLASH_CORRUPTION: return "FAULT_CODE_FLASH_CORRUPTION";
	case FAULT_CODE_FLASH_CORRUPTION_APP_CFG: return "FAULT_CODE_FLASH_CORRUPTION_APP_CFG";
	case FAULT_CODE_FLASH_CORRUPTION_MC_CFG: return "FAULT_CODE_FLASH_CORRUPTION_MC_CFG";
	case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
	case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
	case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
	case FAULT_CODE_UNBALANCED_CURRENTS: return "FAULT_CODE_UNBALANCED_CURRENTS";
	case FAULT_CODE_BRK: return "FAULT_CODE_BRK";
	case FAULT_CODE_RESOLVER_LOT: return "FAULT_CODE_RESOLVER_LOT";
	case FAULT_CODE_RESOLVER_DOS: return "FAULT_CODE_RESOLVER_DOS";
	case FAULT_CODE_RESOLVER_LOS: return "FAULT_CODE_RESOLVER_LOS";
	case FAULT_CODE_ENCODER_NO_MAGNET: return "FAULT_CODE_ENCODER_NO_MAGNET";
	case FAULT_CODE_ENCODER_MAGNET_TOO_STRONG: return "FAULT_CODE_ENCODER_MAGNET_TOO_STRONG";
	case FAULT_CODE_PHASE_FILTER: return "FAULT_CODE_PHASE_FILTER";
	case FAULT_CODE_ENCODER_FAULT: return "FAULT_CODE_ENCODER_FAULT";
	case FAULT_CODE_LV_OUTPUT_FAULT: return "FAULT_CODE_LV_OUTPUT_FAULT";
	}

	return "Unknown fault";
}

Vescmsg *get_rx_frame(void) {
	if (rx_frame_read != rx_frame_write){
		Vescmsg *res = &rx_frames[rx_frame_read++];

		if (rx_frame_read == RX_FRAMES_SIZE){
			rx_frame_read = 0;
		}

		return res;
	} else
		return 0;
}

uint8_t set_rx_frames(Vescmsg* CANRxFrame) {
	uint32_t cmd;

	rx_frames[rx_frame_write++] = *CANRxFrame;
	if (rx_frame_write == RX_FRAMES_SIZE) {
		rx_frame_write = 0;
	}

	cmd = CANRxFrame->Rxmsg.Identifier >> 8;
	if(cmd == (uint32_t)CAN_PACKET_PROCESS_RX_BUFFER || cmd == (uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER)
		return 112;
	else
		return 0;
}
// Added by ibrahim, modified by mohammed
void comm_can2_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	FDCAN_TxMsgEID(&hfdcan2, controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

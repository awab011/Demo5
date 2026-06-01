/*
 * can_bridge.h
 *
 *  Created on: Feb 21, 2026
 *      Author: Ibrahim Meselhy
 */

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include "../../BIOS/COM/FDCAN.h"
#include <stdint.h>
#include <string.h>
#define LASER_POSITION_ID	0x15
#define IMU_YAW_ID          0x16
#define ENCODER_POSITION_ID 0x17
#define MODE_STATUS_ID      0x18

typedef enum {
    MSG_POS,
    MSG_YAW,
    MSG_MODE
} CanBridgeTx;

typedef struct {
    CanBridgeTx type;
    union {
        struct { float x, y; } pos;
        float yaw;
        uint8_t mode;
    } data;
} CanPacket;
enum{
	CAN_MC,
	CAN_MF,
	CAN_ARENA
};
typedef struct{
	float x;
	float y;
}LaserPos;
void Can_Bridge_Transmit(CanPacket pkt);

static inline void Can_Bridge_Tx_Pos(float x, float y) {
    Can_Bridge_Transmit((CanPacket){ .type = MSG_POS, .data.pos = {x, y} });
}

static inline void Can_Bridge_Tx_Yaw(float yaw) {
    Can_Bridge_Transmit((CanPacket){ .type = MSG_YAW, .data.yaw = yaw });
}

static inline void Can_Bridge_Tx_Mode(uint8_t mode) {
    Can_Bridge_Transmit((CanPacket){ .type = MSG_MODE, .data.mode = mode });
}

#endif

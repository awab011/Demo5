/*
 * can_bridge.c
 *
 *  Created on: Feb 21, 2026
 *      Author: Ibrahim Meselhy
 */

#include "can_bridge.h"
//#include "../CAN/can.h"
/*
//#define DISPATCH_2_ARGS(a, b) _Generic((b), \
//    uint8_t:  Can_Bridge_Tx_Mode,           \
//    int:      Can_Bridge_Tx_Mode,           \
//    float:    Can_Bridge_Tx_Yaw,            \
//    double:   Can_Bridge_Tx_Yaw,            \
//    default:  Can_Bridge_Tx_Mode            \
//)(a, b)
//
//#define DISPATCH_3_ARGS(a, b, c) Can_Bridge_Tx_Pos(a, b, c)
//
//#define GET_MACRO(_1, _2, _3, NAME, ...) NAME
//#define Can_Bridge_Tx(...) GET_MACRO(__VA_ARGS__, DISPATCH_3_ARGS, DISPATCH_2_ARGS)(__VA_ARGS__)
*/


void Can_Bridge_Transmit(CanPacket pkt) {
    uint8_t tx_data[8] = {0};
    uint32_t can_id = 0;

    switch (pkt.type) {
        case MSG_POS:
            can_id = ENCODER_POSITION_ID;
            memcpy(tx_data, &pkt.data.pos, 8);
            break;
        case MSG_YAW:
            can_id = IMU_YAW_ID;
            memcpy(tx_data, &pkt.data.yaw, 4);
            break;
        case MSG_MODE:
            can_id = MODE_STATUS_ID;
            tx_data[0] = pkt.data.mode;
            break;
    }

    FDCAN_TxMsg(&hfdcan1, can_id, tx_data, 8);
}

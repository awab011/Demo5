/**
 * @file    r2_can_protocol.h
 * @brief   Single source for laptop <-> H7 CAN frame layouts.
 *
 * Wire layouts MUST stay in sync with the laptop AggregatorNode
 * (Demo5_laptop/src/aggregator_node/aggregator_node/AggregatorNode.py).
 * If you change anything here, change there too.
 *
 * Endianness: all frames are little-endian EXCEPT R2_R1Status (big-endian
 * for x_mm/y_mm). Read R1 fields via __builtin_bswap16().
 */

#ifndef R2_CAN_PROTOCOL_H_
#define R2_CAN_PROTOCOL_H_

#include <stdint.h>

/* ---- CAN IDs (laptop -> H7 unless noted) ---- */
#define CAN_ID_R2_PATH        0x100   /* multi-frame r2can path packet */
#define CAN_ID_FOREST1        0x105
#define CAN_ID_FOREST2        0x106
#define CAN_ID_R1_STATUS      0x107
#define CAN_ID_R2_POSITION    0x108

/* ---- Status flag bits (R2_Forest2Frame_t.status_flags) ---- */
#define R2_STATUS_LIDAR_OK    (1u << 0)
#define R2_STATUS_CAMERA_OK   (1u << 1)
#define R2_STATUS_END         (1u << 2)

/* ---- KFS class codes (per forest block) ---- */
#define R2_KFS_EMPTY          0
#define R2_KFS_R1             1
#define R2_KFS_R2             2
#define R2_KFS_FAKE           9

/* ---- Frame layouts (all 8 bytes, classic CAN) ---- */

/* 0x108 - R2 pose. Little-endian. yaw is in centidegrees. */
typedef struct __attribute__((packed)) {
    int16_t x_mm;
    int16_t y_mm;
    int16_t z_mm;
    int16_t yaw_cdeg;
} R2_PoseFrame_t;

/* 0x105 - Forest blocks 1..8 (zero-indexed 0..7). One KFS class byte each. */
typedef struct __attribute__((packed)) {
    uint8_t blocks[8];
} R2_Forest1Frame_t;

/* 0x106 - Forest blocks 9..12 + current_block + status flags. */
typedef struct __attribute__((packed)) {
    uint8_t blocks[4];      /* blocks 9..12 (zero-indexed 8..11) */
    uint8_t current_block;
    uint8_t status_flags;   /* see R2_STATUS_* */
    uint8_t _pad[2];
} R2_Forest2Frame_t;

/* 0x107 - R1 status. x_mm and y_mm are BIG-ENDIAN on the wire. */
typedef struct __attribute__((packed)) {
    int16_t x_mm_be;        /* read via __builtin_bswap16 */
    int16_t y_mm_be;
    uint8_t zone;
    uint8_t _pad[3];
} R2_R1StatusFrame_t;

#endif /* R2_CAN_PROTOCOL_H_ */

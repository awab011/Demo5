/**
 * @file    r2_world_state.h
 * @brief   Laptop-derived world state on the H7.
 *
 * Two structs, deliberately separate:
 *
 *   g_raw   - raw integers as they arrive from the laptop on CAN.
 *             Writers run in FDCAN3 ISR context (callbacks.c).
 *             Readers must use a critical section to avoid torn reads.
 *             Volatile qualifies every field.
 *
 *   g_world - cooked, ready-to-use values in native units.
 *             Writer is fivems_Task only.
 *             Readers are app tasks. No volatile needed (single writer,
 *             atomic 32-bit reads on Cortex-M7).
 *
 * Wire layout for the raw fields lives in r2_can_protocol.h.
 */

#ifndef R2_WORLD_STATE_H_
#define R2_WORLD_STATE_H_

#include <stdint.h>
#include "main.h"   /* for R1_Status_t, R1_Zone_t */

/* Written in ISR context. Snapshot atomically before reading. */
typedef struct {
    /* R2 pose */
    int16_t r2_pose_x_mm;
    int16_t r2_pose_y_mm;
    int16_t r2_pose_z_mm;
    int16_t r2_yaw_cdeg;        /* centidegrees */

    /* R1 status */
    int16_t r1_pose_x_mm;
    int16_t r1_pose_y_mm;
    uint8_t r1_zone;            /* see R1_Zone_t */

    /* Forest (KFS class per block; see R2_KFS_* in r2_can_protocol.h) */
    uint8_t forest[12];
    uint8_t current_block;      /* 0 = outside grid, 1..12 = on a block */
    uint8_t status_flags;       /* see R2_STATUS_* in r2_can_protocol.h */
} R2RawState_t;

/* Updated by fivems_Task. App tasks read these. */
typedef struct {
    /* R2 pose, native units */
    float r2_x_m;
    float r2_y_m;
    float r2_z_m;
    float r2_yaw_deg;

    /* R1 status (x, y in meters, zone as enum) */
    R1_Status_t r1;

    /* Forest grid mirror (copied from raw under critical section). */
    uint8_t forest[12];
    uint8_t current_block;
    uint8_t status_flags;
} R2WorldState_t;

extern volatile R2RawState_t g_raw;
extern R2WorldState_t        g_world;

#endif /* R2_WORLD_STATE_H_ */

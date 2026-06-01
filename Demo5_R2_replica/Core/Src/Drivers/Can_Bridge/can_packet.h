/*
 * can_packet.h
 *
 *  Created on: Mar 20, 2026
 *      Author: Ibrahim Meselhy
 *
 *
 * ── BLOCK HEIGHT TABLE FOR RED SIDE ────────────────────────────────────────────────────────
 *  Dark green  200mm : B2,  B4,  B10, B12  (0-indexed: 1, 3,  9, 11)
 *  Light green 400mm : B1,  B3,  B5,  B7,  B9,  B11 (0-indexed: 0, 2, 4, 6, 8, 10)
 *  Yellowish   600mm : B6,  B8   (0-indexed: 5, 7)
 *
 * ── HEIGHT DELTA ENCODING  ───────────────────────
 *  delta = target_block_height_mm - current_block_height_mm
 *  Encoded as R2CAN_HDELTA_* constants (0-4):
 *    enc 0 = -400mm   enc 1 = -200mm   enc 2 = 0mm   enc 3 = +200mm   enc 4 = +400mm
 *
 * ── FRAME LAYOUT ────────────────────────
 *
 *  FRAME 0 — Header
 *
 *	B0 = 0xA2, B1 = TOTAL_FRAMES, B2 = TOTAL_COOST, B3 = META, B4 = GR0, B5 = GR1, B6 = GR2, B7 = CRC8
 *
 *
 *  FRAME N — Step frame

 *	B0 = FRAME_IDX, B1 = STEPS_IN_FRAME, B2 = S0_H, B3 = S0_L, B4 = S1_H, B5 = HDELTA[7:2] [7:5] = S0 - [4:2] = S1 - [1:0] = 0x00, B7 = CRC8
 *
 *  STEP ENCODING :
 *  [15:14] action_type   [13:10] current_block (0=outside,1-12=block)   [9:6] target_block-1
 *  [5:4]   direction     [3:2]   collected_after   [1] r1_clear  [0] auto_pickup
 */

#ifndef CAN_PACKET_H
#define CAN_PACKET_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ───────────────────────────────────────────────────────────── */
#define R2CAN_MSG_ID            0xA2u
#define R2CAN_FRAME_BYTES       8u
#define R2CAN_STEPS_PER_FRAME   2u
#define R2CAN_MAX_STEPS         16u
#define R2CAN_MAX_FRAMES        9u

/* Action types */
#define R2CAN_ACT_MOVE          0u
#define R2CAN_ACT_PICKUP        1u
#define R2CAN_ACT_PRE_ENTRY     2u

/* Directions */
#define R2CAN_DIR_DOWN          0u
#define R2CAN_DIR_RIGHT         1u
#define R2CAN_DIR_UP            2u
#define R2CAN_DIR_LEFT          3u

/* Box states */
#define R2CAN_BOX_EMPTY         0u
#define R2CAN_BOX_R1            1u
#define R2CAN_BOX_R2            2u
#define R2CAN_BOX_FAKE          3u

/* ── Block height lookup (mm) ────────────────────────────────────────────── */
/* Index = block-1 (0-based). Use R2CAN_BlockHeight(block_1indexed).         */
#define R2CAN_BLOCK_HEIGHT_MM(b1)  (r2can_block_heights[(b1) - 1u])
extern const uint16_t r2can_block_heights[12];
/*  B1=400 B2=200 B3=400 B4=200 B5=400 B6=600
 *  B7=400 B8=600 B9=400 B10=200 B11=400 B12=200  */

/* ── Height delta encoding ───────────────────────────────────────────────── */
/* Signed delta in mm = target_height - current_height                       */
/* Only 5 distinct values are possible given the 3 height tiers:             */
#define R2CAN_HDELTA_NEG400     0u   /* -400mm 			  */
#define R2CAN_HDELTA_NEG200     1u   /* -200mm  		  */
#define R2CAN_HDELTA_ZERO       2u   /*    0mm			  */
#define R2CAN_HDELTA_POS200     3u   /* +200mm			  */
#define R2CAN_HDELTA_POS400     4u   /* +400mm			  */
#define R2CAN_HDELTA_POS600     5u   /* +600mm			  */
#define R2CAN_HDELTA_INVALID    7u   /* reserved / error  */

/* Compute height delta encoding from two 1-indexed block numbers.          */
/* Returns R2CAN_HDELTA_INVALID if either block is out of range.            */
uint8_t R2CAN_HeightDeltaEnc(uint8_t current_block, uint8_t target_block);

/* PRE_ENTRY variant: robot is on the ground (0mm), so delta = absolute     */
/* height of the target block. Returns POS200, POS400, or POS600.           */
uint8_t R2CAN_PreEntryHeightDeltaEnc(uint8_t target_block);

/* Decode height delta encoding to signed mm value.                         */
int16_t R2CAN_HeightDeltaMm(uint8_t enc);

/* ── Return codes ────────────────────────────────────────────────────────── */
#define R2CAN_WAITING           0
#define R2CAN_COMPLETE          1
#define R2CAN_ERR_CRC          -1
#define R2CAN_ERR_BAD_ID       -2
#define R2CAN_ERR_OVERFLOW     -3
#define R2CAN_ERR_INCOMPLETE   -4
#define R2CAN_ERR_INVALID      -5

/* ── Data structures ──────────────────────────────────────────────────────── */

typedef struct {
    uint8_t action_type;        /* R2CAN_ACT_*                              */
    uint8_t current_block;      /* 0=outside grid, 1-12=1-indexed block     */
    uint8_t target_block;       /* 1-indexed (1-12)                         */
    uint8_t direction;          /* R2CAN_DIR_*                              */
    uint8_t collected_after;    /* 0-3                                      */
    uint8_t requires_r1_clear;  /* 0 or 1                                   */
    uint8_t auto_pickup;        /* 0 or 1                                   */
    uint8_t height_delta_enc;   /* R2CAN_HDELTA_* (pickup/auto_pickup) */
    int16_t height_delta_mm;    /* decoded delta in mm (read-only out) */
} R2CAN_Step;

typedef struct {
    uint8_t    total_cost;
    uint8_t    entry_block;         /* 1-3   */
    uint8_t    exit_block;          /* 10-12 */
    uint8_t    grid_state[12];      /* R2CAN_BOX_* per block (0-indexed)    */
    uint8_t    pre_entry_count;
    uint8_t    step_count;
    R2CAN_Step steps[R2CAN_MAX_STEPS];
} R2CAN_Path;

typedef struct {
    uint8_t data[R2CAN_MAX_FRAMES][R2CAN_FRAME_BYTES];
    uint8_t count;
} R2CAN_Frames;


/* CRC-8/SMBUS (poly=0x07, init=0x00) */
uint8_t R2CAN_CRC8(const uint8_t *buf, uint8_t len);

/**
 * R2CAN_ValidatePath — check rules before packing/transmitting.
 * Rules checked:
 *   - FAKE box not on entry blocks 1, 2, 3
 *   - step counts in range
 * @return R2CAN_OK (0) if valid, R2CAN_ERR_INVALID (-5) if not.
 */
int R2CAN_ValidatePath(const R2CAN_Path *path);

/**
 * R2CAN_Pack — encode path into CAN frames.
 * Automatically fills height_delta_enc for each step.
 * Call R2CAN_ValidatePath first.
 */
void R2CAN_Pack(const R2CAN_Path *path, R2CAN_Frames *out);

/**
 * R2CAN_Unpack — decode CAN frames into path.
 * Fills both height_delta_enc and height_delta_mm on each step.
 */
int R2CAN_Unpack(const R2CAN_Frames *frames, R2CAN_Path *out);

/**
 * R2CAN_FeedFrame — stateful single-frame receiver.
 * Call from HAL_CAN_RxFifo0MsgPendingCallback for every frame on CAN ID 0x100.
 */
int R2CAN_FeedFrame(uint32_t rx_id, const uint8_t *data, uint8_t dlc,
                    R2CAN_Path *out);

/** R2CAN_Reset — clear internal receiver state. */
void R2CAN_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PACKET_H */

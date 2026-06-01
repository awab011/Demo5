/*
 * can_packet.c
 *
 *  Created on: Mar 20, 2026
 *      Author: Ibrahim Meselhy
 *
 */

#include "can_packet.h"

/* ── Block height table (mm, 1-indexed block → 0-indexed array) ─────────── */
/*  B1=400  B2=200  B3=400  B4=200  B5=400  B6=600
 *  B7=400  B8=600  B9=400  B10=200 B11=400 B12=200                          */
const uint16_t r2can_block_heights[12] = {
    400, 200, 400, 200, 400, 600,
    400, 600, 400, 200, 400, 200
};

/* ── Height delta encode / decode ───────────────────────────────────────── */

uint8_t R2CAN_HeightDeltaEnc(uint8_t cur, uint8_t tgt)
{
    int16_t delta;
    if (cur < 1u || cur > 12u || tgt < 1u || tgt > 12u)
        return R2CAN_HDELTA_INVALID;
    delta = (int16_t)r2can_block_heights[tgt - 1u]
          - (int16_t)r2can_block_heights[cur - 1u];
    switch (delta) {
        case -400: return R2CAN_HDELTA_NEG400;
        case -200: return R2CAN_HDELTA_NEG200;
        case    0: return R2CAN_HDELTA_ZERO;
        case  200: return R2CAN_HDELTA_POS200;
        case  400: return R2CAN_HDELTA_POS400;
        default:   return R2CAN_HDELTA_INVALID;
    }
}

/* PRE_ENTRY: robot is on the ground (0mm).
 * Delta = absolute height of target block.
 * 200mm→POS200, 400mm→POS400, 600mm→POS600 (enc=5, reserved slot).        */
uint8_t R2CAN_PreEntryHeightDeltaEnc(uint8_t tgt)
{
    if (tgt < 1u || tgt > 12u) return R2CAN_HDELTA_ZERO;
    switch (r2can_block_heights[tgt - 1u]) {
        case 200: return R2CAN_HDELTA_POS200;  /* +200mm */
        case 400: return R2CAN_HDELTA_POS400;  /* +400mm */
        case 600: return R2CAN_HDELTA_POS600;  /* +600mm */
        default:  return R2CAN_HDELTA_ZERO;
    }
}

int16_t R2CAN_HeightDeltaMm(uint8_t enc)
{
    switch (enc) {
        case R2CAN_HDELTA_NEG400: return -400;
        case R2CAN_HDELTA_NEG200: return -200;
        case R2CAN_HDELTA_ZERO:   return    0;
        case R2CAN_HDELTA_POS200: return  200;
        case R2CAN_HDELTA_POS400: return  400;
        case R2CAN_HDELTA_POS600: return  600;
        default:                  return    0; /* safe default */
    }
}

/* ── CRC-8/SMBUS (poly=0x07, init=0x00) ────────────────────────────────── */
uint8_t R2CAN_CRC8(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0x00u, i, b;
    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (b = 0; b < 8u; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0x07u)
                                : (uint8_t)(crc << 1u);
    }
    return crc;
}

/* ── Validation ─────────────────────────────────────────────────────────── */
int R2CAN_ValidatePath(const R2CAN_Path *path)
{
    uint8_t i;
    /* FAKE box must not appear on entry blocks (0,1,2 = B1,B2,B3) */
    for (i = 0u; i < 3u; i++) {
        if (path->grid_state[i] == R2CAN_BOX_FAKE)
            return R2CAN_ERR_INVALID;
    }
    /* Step count in range */
    if (path->step_count > R2CAN_MAX_STEPS)
        return R2CAN_ERR_INVALID;
    return R2CAN_WAITING; /* == 0 == OK */
}

/* ── Grid pack / unpack ─────────────────────────────────────────────────── */
static void packGrid(const uint8_t grid[12], uint8_t out[3])
{
    uint8_t i, byte_i, bit_i;
    out[0] = 0u; out[1] = 0u; out[2] = 0u;
    for (i = 0u; i < 12u; i++) {
        byte_i = (uint8_t)((i * 2u) / 8u);
        bit_i  = (uint8_t)((i * 2u) % 8u);
        out[byte_i] |= (uint8_t)((grid[i] & 0x03u) << bit_i);
    }
}

static void unpackGrid(const uint8_t in[3], uint8_t grid[12])
{
    uint8_t i, byte_i, bit_i;
    for (i = 0u; i < 12u; i++) {
        byte_i  = (uint8_t)((i * 2u) / 8u);
        bit_i   = (uint8_t)((i * 2u) % 8u);
        grid[i] = (in[byte_i] >> bit_i) & 0x03u;
    }
}

/* ── Step encode / decode (16-bit word, unchanged) ──────────────────────── */
static uint16_t encodeStep(const R2CAN_Step *s)
{
    uint16_t w = 0u;
    w |= (uint16_t)((s->action_type            & 0x03u) << 14u);
    /* current_block: 0 = outside grid, 1-12 = block. Store as-is in 4 bits (0-12 fits). */
    w |= (uint16_t)((s->current_block          & 0x0Fu) << 10u);
    w |= (uint16_t)(((s->target_block  - 1u)   & 0x0Fu) <<  6u);
    w |= (uint16_t)((s->direction              & 0x03u) <<  4u);
    w |= (uint16_t)((s->collected_after        & 0x03u) <<  2u);
    w |= (uint16_t)((s->requires_r1_clear      & 0x01u) <<  1u);
    w |= (uint16_t)((s->auto_pickup            & 0x01u)        );
    return w;
}

static void decodeStep(uint16_t w, R2CAN_Step *s)
{
    s->action_type       = (uint8_t)((w >> 14u) & 0x03u);
    /* current_block stored as-is: 0 = outside, 1-12 = block number */
    s->current_block     = (uint8_t)((w >> 10u) & 0x0Fu);
    s->target_block      = (uint8_t)(((w >>  6u) & 0x0Fu) + 1u);
    s->direction         = (uint8_t)((w >>  4u) & 0x03u);
    s->collected_after   = (uint8_t)((w >>  2u) & 0x03u);
    s->requires_r1_clear = (uint8_t)((w >>  1u) & 0x01u);
    s->auto_pickup       = (uint8_t)((w        ) & 0x01u);
}

/* ── R2CAN_Pack ─────────────────────────────────────────────────────────── */
void R2CAN_Pack(const R2CAN_Path *path, R2CAN_Frames *out)
{
    uint8_t step_frames, total_frames;
    uint8_t entry_enc, exit_enc, pre_enc;
    uint8_t grid_packed[3];
    uint8_t si, fi, s, steps_here;
    uint16_t w;
    uint8_t *fn;
    uint8_t h0_enc, h1_enc;

    memset(out, 0, sizeof(R2CAN_Frames));

    step_frames  = (uint8_t)((path->step_count + R2CAN_STEPS_PER_FRAME - 1u)
                              / R2CAN_STEPS_PER_FRAME);
    total_frames = (uint8_t)(1u + step_frames);

    /* ── Frame 0: Header ─────────────────────────────────────────────────── */
    out->data[0][0] = R2CAN_MSG_ID;
    out->data[0][1] = total_frames;
    out->data[0][2] = path->total_cost;

    entry_enc = (uint8_t)((path->entry_block - 1u) & 0x03u);
    exit_enc  = (uint8_t)((path->exit_block  - 10u) & 0x03u);
    pre_enc   = (uint8_t)(path->pre_entry_count & 0x07u);
    out->data[0][3] = (uint8_t)((exit_enc << 6u) | (entry_enc << 4u) | (pre_enc << 1u));

    packGrid(path->grid_state, grid_packed);
    out->data[0][4] = grid_packed[0];
    out->data[0][5] = grid_packed[1];
    out->data[0][6] = grid_packed[2];
    out->data[0][7] = R2CAN_CRC8(out->data[0], 7u);
    out->count = 1u;

    /* ── Step frames ─────────────────────────────────────────────────────── */
    si = 0u;
    for (fi = 1u; fi <= step_frames && fi < R2CAN_MAX_FRAMES; fi++) {
        fn = out->data[fi];
        fn[0] = fi;
        steps_here = 0u;
        h0_enc = R2CAN_HDELTA_ZERO;
        h1_enc = R2CAN_HDELTA_ZERO;

        for (s = 0u; s < R2CAN_STEPS_PER_FRAME && si < path->step_count; s++, si++) {
            const R2CAN_Step *sp = &path->steps[si];
            w = encodeStep(sp);
            fn[2u + s * 2u] = (uint8_t)(w >> 8u);
            fn[3u + s * 2u] = (uint8_t)(w & 0xFFu);

            /* Compute height delta for this step.
             * Relevant when action == PICKUP or auto_pickup == 1.
             * For MOVE without auto_pickup: delta describes terrain change
             * We always fill it from the block height table.             */
            /* Height delta:
             * current_block==0 means robot is on the ground (pre-entry or ENTER step)
             * use absolute block height of target.
             * Otherwise use relative delta between current and target block heights. */
            {
                uint8_t enc = (sp->current_block == 0u)
                    ? R2CAN_PreEntryHeightDeltaEnc(sp->target_block)
                    : R2CAN_HeightDeltaEnc(sp->current_block, sp->target_block);
                if (s == 0u) h0_enc = enc;
                else         h1_enc = enc;
            }

            steps_here++;
        }

        fn[1] = steps_here;
        /* B6: [7:5]=S0 height enc, [4:2]=S1 height enc, [1:0]=0x00 */
        fn[6] = (uint8_t)(((h0_enc & 0x07u) << 5u) | ((h1_enc & 0x07u) << 2u));
        fn[7] = R2CAN_CRC8(fn, 7u);
        out->count++;
    }
}

/* ── R2CAN_Unpack ───────────────────────────────────────────────────────── */
int R2CAN_Unpack(const R2CAN_Frames *frames, R2CAN_Path *out)
{
    const uint8_t *f0, *fn;
    uint8_t meta, grid_packed[3];
    uint8_t fi, s, steps_here;
    uint8_t b6, h0_enc, h1_enc;
    uint16_t w;

    if (frames->count == 0u) return R2CAN_ERR_INCOMPLETE;

    f0 = frames->data[0];
    if (f0[0] != R2CAN_MSG_ID)       return R2CAN_ERR_BAD_ID;
    if (R2CAN_CRC8(f0, 7u) != f0[7]) return R2CAN_ERR_CRC;
    if (frames->count != f0[1])       return R2CAN_ERR_INCOMPLETE;

    memset(out, 0, sizeof(R2CAN_Path));

    out->total_cost = f0[2];
    meta = f0[3];
    out->entry_block     = (uint8_t)(((meta >> 4u) & 0x03u) + 1u);
    out->exit_block      = (uint8_t)(((meta >> 6u) & 0x03u) + 10u);
    out->pre_entry_count = (uint8_t)((meta >> 1u) & 0x07u);

    grid_packed[0] = f0[4];
    grid_packed[1] = f0[5];
    grid_packed[2] = f0[6];
    unpackGrid(grid_packed, out->grid_state);

    out->step_count = 0u;

    for (fi = 1u; fi < frames->count; fi++) {
        fn = frames->data[fi];
        if (fn[0] != fi)                     return R2CAN_ERR_BAD_ID;
        if (R2CAN_CRC8(fn, 7u) != fn[7])    return R2CAN_ERR_CRC;

        steps_here = fn[1];
        if (steps_here > R2CAN_STEPS_PER_FRAME) return R2CAN_ERR_OVERFLOW;

        /* Extract height deltas from B6 */
        b6     = fn[6];
        h0_enc = (b6 >> 5u) & 0x07u;
        h1_enc = (b6 >> 2u) & 0x07u;

        for (s = 0u; s < steps_here; s++) {
            if (out->step_count >= R2CAN_MAX_STEPS) break;
            w = (uint16_t)(((uint16_t)fn[2u + s * 2u] << 8u) | fn[3u + s * 2u]);
            decodeStep(w, &out->steps[out->step_count]);

            /* Fill height delta */
            out->steps[out->step_count].height_delta_enc =
                (s == 0u) ? h0_enc : h1_enc;
            out->steps[out->step_count].height_delta_mm  =
                R2CAN_HeightDeltaMm((s == 0u) ? h0_enc : h1_enc);

            out->step_count++;
        }
    }

    return R2CAN_COMPLETE;
}

/* ── R2CAN_FeedFrame (stateful receiver) ────────────────────────────────── */
#define R2CAN_LISTEN_ID  0x100u

static R2CAN_Frames s_frames;
static uint8_t      s_expected;
static uint8_t      s_receiving;

void R2CAN_Reset(void)
{
    memset(&s_frames, 0, sizeof(s_frames));
    s_expected  = 0u;
    s_receiving = 0u;
}

int R2CAN_FeedFrame(uint32_t rx_id, const uint8_t *data, uint8_t dlc,
                    R2CAN_Path *out)
{
    uint8_t fi;
    if (rx_id != R2CAN_LISTEN_ID)       return R2CAN_ERR_BAD_ID;
    if (dlc   != R2CAN_FRAME_BYTES)     return R2CAN_ERR_BAD_ID;

    if (data[0] == R2CAN_MSG_ID) {
        memset(&s_frames, 0, sizeof(s_frames));
        s_expected  = data[1];
        s_receiving = 1u;

        if (s_expected > R2CAN_MAX_FRAMES) {
            s_receiving = 0u; return R2CAN_ERR_OVERFLOW;
        }
        if (R2CAN_CRC8(data, 7u) != data[7]) {
            s_receiving = 0u; return R2CAN_ERR_CRC;
        }
        memcpy(s_frames.data[0], data, R2CAN_FRAME_BYTES);
        s_frames.count = 1u;

        if (s_expected == 1u) {
            s_receiving = 0u; return R2CAN_Unpack(&s_frames, out);
        }
        return R2CAN_WAITING;
    }

    if (!s_receiving) return R2CAN_ERR_BAD_ID;

    fi = data[0];
    if (fi == 0u || fi >= R2CAN_MAX_FRAMES) {
        R2CAN_Reset(); return R2CAN_ERR_OVERFLOW;
    }
    if (R2CAN_CRC8(data, 7u) != data[7]) {
        R2CAN_Reset(); return R2CAN_ERR_CRC;
    }

    memcpy(s_frames.data[fi], data, R2CAN_FRAME_BYTES);
    if ((uint8_t)(fi + 1u) > s_frames.count)
        s_frames.count = (uint8_t)(fi + 1u);

    if (s_frames.count == s_expected) {
        s_receiving = 0u; return R2CAN_Unpack(&s_frames, out);
    }
    return R2CAN_WAITING;
}

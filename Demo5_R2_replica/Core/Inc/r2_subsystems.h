/**
 * @file    r2_subsystems.h
 * @brief   Type definitions for the R2 subsystem state machines.
 *
 * MC  - manual-control / spear pickup state machine
 * KFS - Kung Fu Scroll pickup / placement state machine
 * Ext - extension-leg climb / descend state machine
 * MF  - Meihua-Forest alignment helper
 *
 * Variable instances (MC, KFS, Ext, MF_Align, ...) and the data tables
 * that drive them (block_walls, sensor_dir, entry_point, spear_point)
 * remain in main.c. Function prototypes for tasks live in main.h.
 *
 * Also here:
 *   - PP_Points_t (used by entry/spear point arrays)
 *   - DIR_x and SENSOR_x bitmask defines (used by block_walls / sensor_dir)
 *   - AppStatus_t / AppCmd_t (consumed by PathApp_t in main.c)
 *
 * Not here:
 *   - PathApp_t (depends on R2CAN_Path from can_packet.h, kept in main.c)
 */

#ifndef R2_SUBSYSTEMS_H_
#define R2_SUBSYSTEMS_H_

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * Path Planning Points (used by entry_point[], spear_point[])
 * ============================================================ */
typedef struct {
    float x;
    float y;
    float z;
} PP_Points_t;

/* ============================================================
 * MC State Machine (manual control / spear pickup)
 * ============================================================ */
typedef enum {
    MC_CMD_IDLE,
    MC_CMD_PICK,
    MC_CMD_GOTO_ENTRY,
    MC_CMD_BUSY,
    MC_CMD_ERR
} MC_Cmd_t;

typedef enum {
    MC_PICK_IDLE,
    MC_PICK_Start,           /* Robot path-plans to the first point */
    MC_PICK_CLEAR_SENSOR,
    MC_PICK_ARM_OUT,         /* Arm rotates to 270 deg */
    MC_PICK_CONFIRMING,      /* Read IR sensor; advance if spearhead absent */
    MC_PICK_MOVE_NEXT,
    MC_PICK_GRAB,            /* Robot moves into spear rack to grab */
    MC_PICK_PICKUP,          /* Robot rotates spearhead */
    MC_PICK_MOVE,            /* Travel to assembly point */
    MC_PICK_ASSEMBLE,        /* Lock position + start spear-picker rotation */
    MC_PICK_TRANSPORT,
    MC_PICK_ASSEMBLE_POS,
    MC_PICK_End,
    MC_PICK_ERR
} MC_Pick_t;

typedef struct {
    MC_Cmd_t  cmd;
    MC_Pick_t pick_state;
    float     timer;
    bool      assembled;
    uint8_t   target_point;
    float     assemble_time;
    float     pick_time;
} MC_Handle_t;

/* ============================================================
 * KFS State Machine (Kung Fu Scroll pickup / placement)
 * ============================================================ */
typedef enum {
    KFS_LEVEL_INVALID,
    KFS_LEVEL_BELOW,
    KFS_LEVEL_GROUND,
    KFS_LEVEL_20,
    KFS_LEVEL_40,
    KFS_LEVEL_MID,
    KFS_LEVEL_TOP
} KFS_Level_t;

typedef enum {
    PICK_IDLE,
    PICK_Start,              /* Arm at max height, 0 deg, closed grip */
    PICK_ARM_OUT,            /* Arm rotates to 180 deg */
    PICK_START_INTAKE,       /* Open grip + start intake motors */
    PICK_Z_LEVEL,            /* Move to target Z (height_delta_enc) */
    PICK_GRIP,
    PICK_STOP_INTAKE,        /* Stop intake when IR detects KFS */
    PICK_UP_MAX,
    PICK_ARM_IN,             /* Rotate to 0 deg */
    PICK_STORE_LEVEL,        /* Z based on number of stored KFS */
    PICK_STORE_OUT,
    PICK_RELEASE,            /* Skip release for the third KFS */
    PICK_RETURN,
    PICK_End,
    PICK_ERR
} KFS_Pick_t;

typedef enum {
    KFS_CMD_INVALID,
    KFS_CMD_AUTOPICKUP,
    KFS_CMD_PICK,
    KFS_CMD_PLACE,
    KFS_CMD_BUSY,
    KFS_CMD_IDLE
} KFS_Cmd_t;

typedef enum {
    PLACE_IDLE,
    PLACE_Start,
    PLACE_OPEN,
    PLACE_STORE_LEVEL,
    PLACE_GRIP,
    PLACE_UP_MAX,
    PLACE_ARM_OUT_90,
    PLACE_KFS_LEVEL,
    PLACE_ROBOT_EXT,
    PLACE_ARM_OUT,
    PLACE_LOOSEN_GRIP,
    PLACE_KFS_OUT,
    PLACE_ARM_IN_90,
    PLACE_BACK_UP,
    PLACE_ARM_IN,
    PLACE_End,
    PLACE_ERR
} KFS_Place_t;

typedef enum {
    KFS_DIR_INAVLID,
    KFS_DIR_LEFT,
    KFS_DIR_FORWARD,
    KFS_DIR_RIGHT,
    KFS_DIR_BACK
} KFS_DIR_t;

typedef struct {
    KFS_Cmd_t   cmd;
    KFS_Pick_t  pick_state;
    KFS_Place_t place_state;
    KFS_Level_t level;
    uint8_t     auto_pickup;
    KFS_DIR_t   direction;
    int         stored;
    float       timer;
    bool        increment;
} KFS_Handle_t;

/* ============================================================
 * Extension State Machine (climbing / descending blocks)
 * ============================================================ */
typedef enum {
    EXT_LEVEL_INVALID,
    EXT_LEVEL_20,
    EXT_LEVEL_40,
    EXT_LEVEL_MID,
    EXT_LEVEL_TOP
} Ext_Level_t;

typedef enum {
    EXT_CMD_INVALID,
    EXT_CMD_AUTOPICK,
    EXT_CMD_FORWARD,
    EXT_CMD_BACKWARD,
    EXT_CMD_LEFT,
    EXT_CMD_RIGHT,
    EXT_CMD_RAMP,
    EXT_CMD_BUSY,
    EXT_CMD_CONFIRMING,
    EXT_CMD_IDLE
} Ext_Cmd_t;

typedef enum {
    EXT_DIR_INVALID,
    EXT_DIR_LEFT,
    EXT_DIR_FORWARD,
    EXT_DIR_RIGHT,
    EXT_DIR_BACK
} Ext_DIR_t;

typedef enum {
    CLIMB_IDLE,
    CLIMB_Start,             /* Extensions inside the robot by 1 cm */
    CLIMB_UP,                /* Extend up to 200 or 400 mm based on IR */
    CLIMB_FORWARD,           /* Drive ext-wheels until front IR triggers */
    CLIMB_RETRACT_FRONT,
    CLIMB_INTO_BLOCK,        /* Drive forward until rear IR triggers */
    CLIMB_RETRACT_BACK,
    CLIMB_CLEAR_BLOCK,       /* Drive until back IR no longer detects block */
    CLIMB_End,
    CLIMB_ERR
} Ext_Climb_t;

typedef enum {
    DESCEND_IDLE,
    DESCEND_Start,
    DESCEND_EXTEND_FRONT,
    DESCEND_FORWARD,
    DESCEND_STOP,
    DESCEND_EXTEND_REAR,
    DESCEND_CLEAR,
    DESCEND_RETRACT_ALL,
    DESCEND_End,
    DESCEND_ERR
} Ext_Descend_t;

typedef struct {
    Ext_Level_t   level;
    Ext_Cmd_t     cmd;
    Ext_Descend_t descend_state;
    Ext_Climb_t   climb_state;
    Ext_DIR_t     direction;
    float         timer;
    bool          increment;
} Ext_Handle_t;

/* ============================================================
 * Meihua-Forest alignment
 *   DIR_*    : world cardinal directions (used by block_walls)
 *   SENSOR_* : robot-relative sensor positions
 *   Heading_t: which world direction the robot is facing
 * ============================================================ */
#define DIR_N (1 << 0)
#define DIR_W (1 << 1)
#define DIR_S (1 << 2)
#define DIR_E (1 << 3)

#define SENSOR_F (1 << 0)
#define SENSOR_L (1 << 1)
#define SENSOR_B (1 << 2)
#define SENSOR_R (1 << 3)

typedef enum {
    HEADING_NORTH = 0,
    HEADING_WEST,
    HEADING_SOUTH,
    HEADING_EAST
} Heading_t;

typedef struct {
    float*    dist_x;
    float*    dist_y;
    int       dir_x;
    int       dir_y;
    bool      x_aligned;
    bool      y_aligned;
    Heading_t heading;
} MF_Align_t;

/* ============================================================
 * Application-level (PathApp) status / command enums.
 * PathApp_t itself stays in main.c (depends on R2CAN_Path*).
 * ============================================================ */
typedef enum {
    APP_IDLE,
    APP_Start,
    APP_PP,                  /* Path Plan to the entry block */
    APP_BUSY,
    APP_ALIGN,
    APP_COMMAND,
    APP_DONE
} AppStatus_t;

typedef enum {
    APP_CMD_IDLE,
    APP_CMD_Start,
    APP_CMD_BUSY,
    APP_CMD_COMPLETED
} AppCmd_t;

#endif /* R2_SUBSYSTEMS_H_ */

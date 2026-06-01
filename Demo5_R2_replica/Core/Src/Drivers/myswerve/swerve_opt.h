/*
 * swerve.h
 *
 *  Created on: Oct 17, 2025
 *      Author: Ibrahim Meselhy
 */

#ifndef SRC_MYSWERVE_SWERVE_OPT_H_
#define SRC_MYSWERVE_SWERVE_OPT_H_

#include "../../Actuators/VESC_FDCAN/vesc_interface.h"
#include "../../Actuators/Robomaster/robomaster.h"
#include "../../Algorithms/PID/PID.h"
#include "../../BIOS/EXTI.h"
#include "../../Algorithms/MODN/MODN.h"
#include "../../Sensors/PWMEncoder/PWMEncoder.h"
#include <string.h>  // For memset

// ============================================================================
// SWERVE STRUCTURE (OPTIMIZED)
// ============================================================================

typedef struct {
    // Configuration
    int type;                     // FWD_SWERVE or TRI_SWERVE
    int num_wheels;               // 3 or 4 depending on type
    float gear_ratio;

    // Alignment state
    int aligned;
    int align_flag;

    // Wheel control
    float vel[4];                 // Commanded wheel velocities
    float fvel[4];                // Final velocities (with sign correction)
    float sign[4];                // Direction multiplier (+1 or -1)

    // Angle tracking
    float ang[4];                 // Current target angles
    float ang_prev[4];            // Previous angles
    float total_ang[4];           // Unwrapped angles
    float total_ang_prev[4];      // Previous unwrapped angles
    float ang1[4];                // Angle difference option 1
    float ang2[4];                // Angle difference option 2
    float final_ang[4];           // Final commanded angles
    float final_ang_prev[4];      // Previous final angles
    float addvalue[4];            // Unwrapping offset
    float offset[4];

    // State
    int state[4];                    // Optimization state per wheel
    uint32_t timer;                  // Idle timer
    float total_vel;

    // Hardware interfaces
    VESC_t *vesc;                 // Motor controllers
    RBMS_t *rbms;                 // RBMS motors (for wheel steering)

} swerve_t;
extern swerve_t myswerve;

typedef struct {
    // Hardware references
    RBMS_t *rbms;
    int motor_index;              // 0-3 for which motor

    const volatile uint32_t *reg; //register for the Hall(IR sensor)
    uint8_t bit;

    PWMEnc_t *encoder;            // Pointer to encoder (enc1, enc2, etc.)

    // Configuration
    float gear_ratio;
    float target_angle;           // Desired zero angle (usually 0°)

    // Status
    volatile int aligned;         // 1 when aligned
    volatile int error;           // 1 if alignment failed
    float zero_rbms_ang;
    float zero_rbms_current_ang;
    double zero_rbms_current_frac;
    double zero_rbms_frac;
    double zero_rbms_whole;
    // Alignment data
    float zero_angle;
    float zero_rev;
    float zero_offset;
    float final_position;

    // Statistics (for debugging)
    uint32_t align_duration_ms;
    int method_used;              // 0=PWM, 1=RbMaster

} MonoSwerve_t;

extern MonoSwerve_t swerveA , swerveB, swerveC, swerveD;
// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void SwerveInit(int swerve_type, float swerve_gear_ratio, float width,
                float length, VESC_t *vesc, RBMS_t *rbms, swerve_t *swerve);


void MonoSwerve_Init(MonoSwerve_t *mono, RBMS_t *rbms, int motor_idx,
                     const volatile uint32_t* reg, uint8_t bit,
                     PWMEnc_t *encoder, float gear_ratio, float target_angle);

int MonoSwerve_CheckAlignment(MonoSwerve_t *mono);

int MonoSwerve_AlignPWM(MonoSwerve_t *mono);

int MonoSwerve_AlignRbMaster(MonoSwerve_t *mono);

int MonoSwerve_Align(MonoSwerve_t *mono);

void SwerveRun(swerve_t *swerve);


//EXAMPLE USE: uint8_t hall = readbit(&GPIOE->IDR, 15);

static inline uint8_t readbit(const volatile uint32_t *reg, uint8_t bit)
{
    return ((*reg >> bit) & 1U);
}

// ============================================================================
// HELPER FUNCTIONs
// ============================================================================

static inline int MonoSwerve_IsAligned(MonoSwerve_t *mono) {
    return mono->aligned;
}

static inline int MonoSwerve_HasError(MonoSwerve_t *mono) {
    return mono->error;
}

static inline uint32_t MonoSwerve_GetAlignDuration(MonoSwerve_t *mono) {
    return mono->align_duration_ms;
}

static inline void MonoSwerve_ClearError(MonoSwerve_t *mono) {
    mono->error = 0;
}
static inline int Swerve_IsRunning(swerve_t *swerve) {
    return (swerve->total_vel > 0.05) ;
}

#endif /* SRC_MYSWERVE_SWERVE_OPT_H_ */

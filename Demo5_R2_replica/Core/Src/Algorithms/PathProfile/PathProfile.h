/*
 * PathProfile.h
 *
 *  Created on: Sep 27, 2023
 *      Author: yvc
 *      Updated and maintained by: Ibrahim Meselhy
 */

#ifndef SRC_PATHPROFILE_PATHPROFILE_H_
#define SRC_PATHPROFILE_PATHPROFILE_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#define PLANNER_STATE_IDLE 0
#define PLANNER_STATE_START_MOVE 1
#define PLANNER_STATE_WAIT_MOVE 2

// Steepness constant for the S-curve
#define S_CURVE_K_VAL (1.47157f)
// Time offset to start at the "bottom" of the S-curve
#define S_CURVE_START_T_FACTOR (1.2f)
// Time multiplier to ensure the S-curve is 100% complete
#define S_CURVE_END_T_FACTOR (1.5f)

typedef struct {
    uint8_t number_of_point;
//    float points[][4];  if you use this and then put it inside the Path_Profile_t then that structure would have an undefined type
    float (*points)[4];

} Path_Profile_Plan_t;

typedef struct {
    union {
        struct {
            uint8_t state : 4;
            int8_t dir : 2;
            uint8_t error : 1;
        };
        uint8_t status;
    };
    float d, d1, d2, d3, td, v, t, x, path, lastd;
    float k_val;
    float start_t;
    float end_t	;

    uint32_t counter;
    Path_Profile_Plan_t plan;

} Path_Profile_t;
typedef struct {
    // --- Configuration (Set by user) ---
    uint8_t num_of_axis;         // How many axes in this group
    Path_Profile_t **profiles;   // Pointer to an array of profile pointers (e.g., [&profX, &profY])

    // --- Planner's Internal State (Managed by the function) ---
    uint8_t state;               // 0=Idle, 1=Start_Move, 2=Wait_Move
    uint8_t point_number;        // The current move index

} Planner_Group_t;
extern int number_of_profile;
extern Path_Profile_t **P_to_Profile;


void Path_Profile_Init(Path_Profile_t *prof);
void Path_Profile_Inc();
void Path_Profile_Set(Path_Profile_t *prof, float offset, float d1, float td, float d3, float v);
void Path_Profile_Absolute_Set(Path_Profile_t *prof, float d1, float abs_d, float d3, float v);
void Path_Profile_Reset(Path_Profile_t *prof);
void Path_Profile_Update(Path_Profile_t *prof);
void Path_Profile_Planner(Planner_Group_t* group);

#endif /* SRC_PATHPROFILE_PATHPROFILE_H_ */

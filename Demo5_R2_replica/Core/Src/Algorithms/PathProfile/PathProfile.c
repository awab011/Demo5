/*
 * PathProfile.h
 *
 *  Created on: Sep 27, 2023
 *      Author: yvc
 *      Updated and maintained by: Ibrahim Meselhy
 *
 *  https://www.desmos.com/calculator/xirbeskaaz
 */

#include "PathProfile.h"

int number_of_profile = 0;
Path_Profile_t **P_to_Profile = NULL;
/*
 * Function Name		: Path_Profile_Init
 * Function Description : Called to init Path_Profile.
 * Function Remarks		: None
 * Function Arguments	: *prof			,	pointer to structure Path_Profile_t
 * Function Return		: None
 * Function Example		: Path_Profile_Init(&prof1);
 */
void Path_Profile_Init(Path_Profile_t *prof){
	Path_Profile_t **temp_ptr = NULL;

    if (number_of_profile == 0){
//        P_to_Profile = (Path_Profile_t **)malloc(sizeof(Path_Profile_t *));
//        *P_to_Profile = (Path_Profile_t *)malloc(sizeof(Path_Profile_t)); // bug since it is overwritten below
    	temp_ptr = (Path_Profile_t **)malloc(sizeof(Path_Profile_t *));
    }else{
//        P_to_Profile = (Path_Profile_t **)realloc(P_to_Profile, sizeof(Path_Profile_t *) * (number_of_profile + 1));
//        *(P_to_Profile) = (Path_Profile_t *)realloc(*(P_to_Profile), sizeof(Path_Profile_t) * (number_of_profile + 1)); //bug
    	temp_ptr = (Path_Profile_t **)realloc(P_to_Profile, sizeof(Path_Profile_t *) * (number_of_profile + 1));
    }
    if (temp_ptr == NULL) {
            prof->error = 1;
            return;
        }

    P_to_Profile = temp_ptr;

    P_to_Profile[number_of_profile] = prof;

    prof->state = 0;
    prof->error = 0;
    prof->lastd = 0;
    prof->counter = 0;
    prof->k_val = 1.47157f;
    prof->start_t = 1.2f;
    prof->end_t = 1.5f;
    number_of_profile++;
}

/*
 * Function Name		: Path_Profile_Inc
 * Function Description : Called to increment counter of all Path_Profile.
 * Function Remarks		: Call this function in 1ms timer interrupt.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: Path_Profile_Inc();
 */
void Path_Profile_Inc(){
    for (int i = 0; i < number_of_profile; i++){
        if (P_to_Profile[i]->state != 0 && P_to_Profile[i]->error == 0){
			P_to_Profile[i]->counter++;
        }
    }
}


/*
 * Function Name        : Path_Profile_Set
 * Function Description : Called to set Path_Profile.
 * Function Remarks     : Sign of td indicate direction of path.
 *                        td must be greater than d1 + d3.
 * Function Arguments   : *prof       ,   pointer to structure Path_Profile_t
 *                        offset      ,   offset of path
 *                        d1          ,   distance of accel
 *                        td          ,   total distance
 *                        d3          ,   distance of decel
 *                        v           ,   velocity
 * Function Return      : None
 * Function Example     : Path_Profile_Set(&prof1, 0, 1, 4, 2, 5);
 */
void Path_Profile_Set(Path_Profile_t *prof, float offset, float d1, float td, float d3, float v){

    // Check input velocity parameter (not prof->v which isn't set yet!)
    if (fabsf(v) < 1e-6f) {
        prof->error = 1;
        prof->state = 0;
        return;
    }

    prof->lastd = offset;
    prof->path = offset;
    prof->td = td;
    prof->dir = (prof->td >= 0) ? 1 : -1;
    prof->d1 = fabsf(d1) * prof->dir;
    prof->d3 = fabsf(d3) * prof->dir;
    prof->v = fabsf(v) * prof->dir;
    prof->d2 = prof->td - prof->d1 - prof->d3;

    // Check if acceleration + deceleration distances exceed total distance
    if (fabsf(prof->d1) + fabsf(prof->d3) > fabsf(td)){
        prof->error = 1;
        prof->state = 0;  // Also set state to 0 on error for consistency
        return;
    }

    prof->state = 1;
    prof->counter = 0;
}
void Path_Profile_Absolute_Set(Path_Profile_t *prof, float d1, float abs_d, float d3, float v){
	Path_Profile_Set(prof, prof->lastd, d1, abs_d - prof->lastd, d3, v);
}

/*
 * Function Name		: Path_Profile_Reset
 * Function Description : Called to reset Path_Profile.
 * Function Remarks		: None
 * Function Arguments	: *prof			,	pointer to structure Path_Profile_t
 * Function Return		: None
 * Function Example		: Path_Profile_Reset(&prof1);
 */
void Path_Profile_Reset(Path_Profile_t *prof){
    prof->state = 0;
    prof->error = 0;
    prof->lastd = 0;
    prof->counter = 0;
}

/*
 * Function Name		: Path_Profile_Update
 * Function Description : Called to update Path_Profile.
 * Function Remarks		: Can be called in any time frame as it track time by independent counter.
 * Function Arguments	: *prof			,	pointer to structure Path_Profile_t
 * Function Return		: None
 * Function Example		: Path_Profile_Update(&prof1);
 */
void Path_Profile_Update(Path_Profile_t *prof){

	if (fabsf(prof->v) < 1e-6f) {
	    prof->error = 1;
	    prof->state = 0;
	    return;
	}
    switch (prof->state)
    {
    case 0:
        break;
    case 1: // accel
        prof->d = prof->d1;
        prof->t = 2 * prof->d / prof->v;
        prof->x = prof->counter/1000.0 - prof->t * prof->start_t;
        prof->path = prof->lastd + 2 * prof->d / (1 + expf(-prof->k_val / prof->t * prof->x * M_E));
        if (prof->x >= 0){
            prof->lastd += prof->d;
            prof->state = 2;
            prof->counter = 0;
        }
        break;
    case 2: // const
        prof->d = prof->d2;
        prof->t = prof->d / prof->v;
        prof->x = prof->counter/1000.0;
        prof->path = prof->lastd + prof->x * prof->v;
        if (prof->x >= prof->t){
            prof->lastd += prof->d;
            prof->state = 3;
            prof->counter = 0;
        }
        break;
    case 3: // decel
        prof->d = prof->d3;
        prof->t = 2 * prof->d / prof->v;
        prof->x = prof->counter/1000.0;
        prof->path = prof->lastd - prof->d + 2 * prof->d / (1 + expf(-prof->k_val / prof->t * prof->x * M_E));
        if (prof->x >= prof->t * prof->end_t){
            prof->lastd += prof->d;
            prof->path = prof->lastd;
            prof->state = 4;
            prof->counter = 0;
        }
        break;
    case 4:
    	prof->state = 0;
    	break;
    }
}


/*
 * Function Name        : Path_Profile_Planner
 * Function Description : Manages sequential execution of path profiles for multiple axes
 * Function Remarks     : Executes points from each axis's plan sequentially.
 *                        All axes in a group move synchronously through each point.
 *                        Handles errors gracefully by stopping all axes.
 * Function Arguments   : *group      ,   pointer to Planner_Group_t structure
 * Function Return      : None
 * Function Example     : Path_Profile_Planner(&my_group);
 */
void Path_Profile_Planner(Planner_Group_t *group) {

    // Safety checks - validate group structure
    if (group == NULL) {
        return; // Nothing to do if group is null
    }

    if (group->profiles == NULL || group->num_of_axis == 0) {
        group->state = PLANNER_STATE_IDLE;
        return; // Invalid configuration
    }

    // Additional safety: check if first profile is valid
    if (group->profiles[0] == NULL || group->profiles[0]->plan.points == NULL) {
        group->state = PLANNER_STATE_IDLE;
        return;
    }

    // Get the total number of points from the first profile's plan
    // (Assumes all profiles in a group have the same number of points)
    uint8_t num_of_points = group->profiles[0]->plan.number_of_point;

    switch (group->state)
    {
    case PLANNER_STATE_IDLE: // 0
        // Planner is idle. Do nothing.
        // To start execution, set group->state = PLANNER_STATE_START_MOVE from outside.
        break;

    case PLANNER_STATE_START_MOVE: // 1
        // Check if we have more points to execute
        if (group->point_number < num_of_points) {
            uint8_t i = group->point_number; // Current point index

            // Start the move for all axes in the group (single loop - more efficient)
            for (uint8_t a = 0; a < group->num_of_axis; a++) {
                Path_Profile_t *prof = group->profiles[a];

                // Safety check: ensure profile pointer is valid
                if (prof == NULL) {
                    group->state = PLANNER_STATE_IDLE;
                    return; // Abort if any profile is null
                }

                // Safety check: ensure plan data is allocated
                if (prof->plan.points == NULL) {
                    group->state = PLANNER_STATE_IDLE;
                    return; // Abort if plan data is missing
                }

                // Safety check: verify this profile has enough points
                // (Defensive programming - shouldn't happen if all profiles have same length)
                if (i >= prof->plan.number_of_point) {
                    group->state = PLANNER_STATE_IDLE;
                    group->point_number = 0;
                    return;
                }

                // Extract path parameters for this specific axis at point i
                float d1 = prof->plan.points[i][0]; // Acceleration distance
                float td = prof->plan.points[i][1]; // Total distance
                float d3 = prof->plan.points[i][2]; // Deceleration distance
                float v  = prof->plan.points[i][3]; // Velocity

                // Set the path profile for this axis
                Path_Profile_Set(prof, prof->lastd, d1, td, d3, v);

                // Check if Path_Profile_Set encountered an error
                if (prof->error != 0) {
                    // Error in this axis - abort the entire group
                    group->state = PLANNER_STATE_IDLE;
                    return;
                }
            }

            // All axes started successfully, transition to wait state
            group->state = PLANNER_STATE_WAIT_MOVE;
        }
        else {
            // No more points to execute. Plan is complete.
            group->state = PLANNER_STATE_IDLE;
            group->point_number = 0; // Reset for next execution cycle
        }
        break;

    case PLANNER_STATE_WAIT_MOVE: // 2
        {
            // Wait for all profiles to finish their current move
            uint8_t all_idle = 1;  // Assume all are idle until proven otherwise
            uint8_t any_error = 0; // Track if any axis has an error

            // Check status of all axes
            for (uint8_t a = 0; a < group->num_of_axis; a++) {
                Path_Profile_t *prof = group->profiles[a];

                // Safety check for null pointer
                if (prof == NULL) {
                    group->state = PLANNER_STATE_IDLE;
                    return;
                }

                // Check if this axis is still moving
                if (prof->state != 0) {
                    all_idle = 0; // At least one axis is still moving
                }

                // Check for errors on this axis
                if (prof->error != 0) {
                    any_error = 1; // Error detected
                }
            }

            // Handle errors with priority
            if (any_error != 0) {
                // An axis encountered an error - stop the entire group
                group->state = PLANNER_STATE_IDLE;
                // Optional: Set a dedicated error state or flag if needed
                // group->error_flag = 1;
            }
            else if (all_idle) {
                // All axes completed their move successfully
                group->point_number++;                  // Advance to next point
                group->state = PLANNER_STATE_START_MOVE; // Start next move
            }
            // If not all idle and no errors, continue waiting
        }
        break;

    default:
        // Invalid state - reset to idle
        group->state = PLANNER_STATE_IDLE;
        break;
    }
}

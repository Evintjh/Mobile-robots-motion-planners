/* ----------------------------------------------------------------------------
 * Free space templates,
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file motion_primitive.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef FREE_SPACE_MOTION_PRIMITIVE_H
#define FREE_SPACE_MOTION_PRIMITIVE_H


#include <free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
typedef struct mmaneuver_s{
    body_t body;
}mmaneuver_t;
*/

extern const struct MotionPrimitive MotionPrimitiveUnicycle;
struct MotionPrimitive{
    void (*create)(motion_primitive_t*);
    void (*allocate_memory)(motion_primitive_t*); // type of control/kinematic model
    void (*deallocate_memory)(motion_primitive_t*);

    void (*sample)(const motion_primitive_t*, const point2d_t*, 
        double, point2d_array_t*);
    void (*excite)(const void*, double, const pose2d_t*, pose2d_t*);
};

void motion_primitive_unicycle_create(motion_primitive_t* motion_primitive);

void motion_primitive_unicycle_allocate_memory(motion_primitive_t* motion_primitive);

void motion_primitive_unicycle_deallocate_memory(motion_primitive_t* motion_primitive);

void motion_primitive_unicycle_sample(const motion_primitive_t *motion_primitive, 
    const point2d_t *offset, double sampling_interval, point2d_array_t *samples);

void motion_primitive_unicycle_excite(const void *control, 
    double time, const pose2d_t *pose_init, pose2d_t *pose_final);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
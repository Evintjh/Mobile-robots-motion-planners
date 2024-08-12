/* ----------------------------------------------------------------------------
 * Free space motion tubes
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file motion_tube.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_H

#include<stdlib.h>
#include<stdbool.h>

#include <sensor_data_structure/lidar.h>

#include<free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

// Methods are collected in MotionTubeCartesian
extern const struct MotionTube MotionTube;
struct MotionTube{
    void (*create)(motion_tube_t*);
    void (*allocate_memory)(motion_tube_t*, size_t *, uint8_t);
    void (*deallocate_memory)(motion_tube_t*);
    
    void (*sample)(motion_tube_t*, double, const point2d_t *, 
        const motion_primitive_t*, const range_sensor_t*, const pose2d_t*);
    struct{
        void(*availability)(const motion_tube_t*, const lidar_t*, bool*);
    }Monitor;
};

/** MotionTube methods **/

void motion_tube_create(motion_tube_t *motion_tube);

void motion_tube_allocate_memory(motion_tube_t *motion_tube, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE);

void motion_tube_deallocate_memory(motion_tube_t *motion_tube);

void motion_tube_sample(motion_tube_t*, double sampling_interval, const point2d_t *footprint,
    const motion_primitive_t* motion_primitive, const range_sensor_t* range_sensor, 
    const pose2d_t* pose_sensor);

void motion_tube_availability(const motion_tube_t* motion_tube, const lidar_t* lidar, bool* is_available);

/** Generic methods **/

void motion_tube_cartesian_to_sensor_space(const motion_tube_cartesian_t* motion_tube_cartesian,
    const range_sensor_t* range_sensor, const pose2d_t* pose_sensor,
    motion_tube_sensor_space_t* motion_tube_sensor_space);


#ifdef __cplusplus
}  // extern C
#endif

#endif

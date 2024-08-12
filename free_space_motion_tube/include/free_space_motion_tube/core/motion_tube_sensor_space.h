/* ----------------------------------------------------------------------------
 * Free space motion tubes
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file motion_tube_sensor_space.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef FREE_SPACE_MOTION_TUBE_CORE_motion_tube_sensor_space_H
#define FREE_SPACE_MOTION_TUBE_CORE_motion_tube_sensor_space_H

#include<stdlib.h>
#include<stdbool.h>

#include <sensor_data_structure/lidar.h>

#include<free_space_motion_tube/core/basic.h>


#ifdef __cplusplus
extern "C" {
#endif

// Methods are collected in MotionTubeSensorSpace
extern const struct MotionTubeSensorSpace MotionTubeSensorSpace;
struct MotionTubeSensorSpace{
    void (*create)(motion_tube_sensor_space_t*);
    void (*allocate_memory)(motion_tube_sensor_space_t*, size_t);
    void (*deallocate_memory)(motion_tube_sensor_space_t*);
    struct{
        void(*availability)(const motion_tube_sensor_space_t*, 
            const lidar_t*, bool*);
    }Monitor;
};

void motion_tube_sensor_space_create(motion_tube_sensor_space_t *motion_tube);

void motion_tube_sensor_space_allocate_memory(motion_tube_sensor_space_t *motion_tube, 
    size_t max_number_of_samples);

void motion_tube_sensor_space_deallocate_memory(motion_tube_sensor_space_t *motion_tube);

void motion_tube_sensor_space_monitor_availability(
    const motion_tube_sensor_space_t *motion_tube, const lidar_t *lidar, bool *is_available);

#ifdef __cplusplus
}  // extern C
#endif

#endif

/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file free_space_activity.h
 * @date Mar 22, 2022
 **/

#ifndef FREE_SPACE_ACTIVITY_H
#define FREE_SPACE_ACTIVITY_H

// Mutex
#include <pthread.h>

// AACAL
#include <five_c/activity/activity.h>
#include <read_file/read_file.h>

#include <sensor_data_structure/range_scan.h>
#include <sensor_data_structure/range_sensor.h>
#include <geometry_data_structure/pose2d.h>
#include <mechanics_data_structure/body.h>
#include <mechanics_data_structure/velocity.h>
#include <platform_data_structure/kelo_tricycle.h>
#include <navigation_data_structure/range_motion_tube.h>
#include <navigation_data_structure/odometry2d.h>

// Free space
#include <free_space_motion_tube/core/basic.h>
#include <free_space_motion_tube/core/motion_tube.h>

#define MAX_NUMBER_MOTION_TUBE 102

typedef struct free_space_activity_s{
    void (*create_lcsm)(activity_t*, const char* activity_name);
    void (*resource_configure_lcsm)(activity_t*);
    void (*destroy_lcsm)(activity_t*);
}free_space_activity_t;

// Parameters
typedef struct free_space_activity_params_s{
    // input data
    char configuration_file[100]; 
    range_sensor_t *rt_range_sensor, range_sensor;
    range_scan_t *rt_range_scan, range_scan;
    odometry2d_t *rt_odometry, odometry;
    velocity_t *rt_desired_velocity, desired_velocity; 
    velocity_t *rt_command_velocity, command_velocity; 
    
    // platform and sensor params
    kelo_tricycle_t *platform;
    
    // Template
    motion_tube_t motion_tube[3][6][21];
    motion_primitive_t motion_primitive[3][6][21];
    pose2d_t pose_sensor;


    // Parameters
    point2d_t footprint[4];
    double time_horizon[3];
    double sampling_interval[3];
    int max_number_of_samples;

    range_motion_tube_t *rt_range_motion_tube;    
    
    // We will choose the one available one that has  the most similar curvature.
}free_space_activity_params_t;

// Continuous state
typedef struct free_space_activity_continuous_state_s{
    // control commands
    velocity_t des_platform_velocity;
    range_motion_tube_t range_motion_tube;
}free_space_activity_continuous_state_t;

// Discrete state
typedef struct free_space_activity_discrete_state_s{
}free_space_activity_discrete_state_t;

typedef struct free_space_coordination_state_s {
    // Coordination flags
    bool *execution_request;
    bool *deinitialisation_request;
    // Coordination with other activities
    bool *platform_control_ready, *platform_control_dead;
    bool *lidar_ready, *lidar_dead; 
    // Mutex
    pthread_mutex_t *range_scan_lock, *platform_control_lock, *odometry_lock;
    pthread_mutex_t *desired_velocity_lock, *motion_tube_lock;
} free_space_activity_coordination_state_t;

extern const free_space_activity_t ec_free_space_activity;

void configure_free_space_activity_from_file(const char *file_path, 
    free_space_activity_params_t *params, int *status);
    
#endif //FREE_SPACE_ACTIVITY_H


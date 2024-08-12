#ifndef DRIVE_DATA_STRUCTURE_DIFFERENTIAL_DRIVE_H
#define DRIVE_DATA_STRUCTURE_DIFFERENTIAL_DRIVE_H

#include <sensor_data_structure/differential_drive_sensor.h>
#include <actuation_data_structure/differential_drive_actuation.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct differential_drive_params_s{
    double wheel_radius;  ///< radius of wheels (assumed to be the same for both wheels)
    double wheel_track;  ///< distance between left and right wheels 
    double pivot_offset;  ///< distance from center of the wheels to center of rotation of the kelo base 
}differential_drive_params_t;

typedef struct differential_drive_limits_s{
    double max_wheel_velocity;  ///< velocity in rad/s
    double max_wheel_acceleration;  ///< acceleration in rad/s^2    
}differential_drive_limits_t;

typedef struct differential_drive_config_s{
    differential_drive_params_t params;    
    differential_drive_limits_t limits;
}differential_drive_config_t;

typedef struct differential_drive_s{
    differential_drive_config_t config;
    differential_drive_sensor_t sensor;
    differential_drive_actuation_t actuation;
}differential_drive_t;

#ifdef __cplusplus
}
#endif

#endif
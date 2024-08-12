
#ifndef SENSOR_DATA_STRUCTURE_DIFFERENTIAL_DRIVE_SENSOR_H
#define SENSOR_DATA_STRUCTURE_DIFFERENTIAL_DRIVE_SENSOR_H

#include <time.h>

#include <sensor_data_structure/encoder.h>
#include <sensor_data_structure/accelerometer.h>
#include <sensor_data_structure/gyroscope.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct differential_drive_sensor_s{
    struct timespec timestamp;    
    accelerometer_t accelerometer;
    gyroscope_t gyroscope;
    struct encoder{
        encoder_t left_wheel, right_wheel, pivot;
    }encoder;
    struct temperature{
        double left_wheel, right_wheel;
    }temperature;
    struct voltage{
        double bus, left_wheel, right_wheel;
    }voltage;
}differential_drive_sensor_t;

#ifdef __cplusplus
}
#endif

#endif
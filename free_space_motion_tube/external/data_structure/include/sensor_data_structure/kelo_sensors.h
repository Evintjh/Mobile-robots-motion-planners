#ifndef KELO_SENSORS_H
#define KELO_SENSORS_H

#include <sensor_data_structure/encoder.h>

typedef struct kelo_sensors_s{
    encoder_t left_wheel, right_wheel, pivot;
    float voltage_bus;
    float voltage_1, voltage_1_u, voltage_1_v, voltage_1_w;
    float voltage_2, voltage_2_u, voltage_2_v, voltage_2_w;
}kelo_sensors_t;

#endif

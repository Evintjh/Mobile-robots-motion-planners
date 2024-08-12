#ifndef LIDAR_DATA_STRUCTURE_H
#define LIDAR_DATA_STRUCTURE_H

#include <sensor_data_structure/range_scan.h>
#include <sensor_data_structure/range_sensor.h>

typedef struct lidar_s{
    range_sensor_t *range_sensor;
    range_scan_t *range_scan;
}lidar_t;

#endif
#ifndef DATA_STRUCTURE_UTILS_H
#define DATA_STRUCTURE_UTILS_H

#include <string.h>

#include <drive_data_structure/differential_drive.h>
#include <navigation_data_structure/odometry2d.h>

#ifdef __cplusplus
extern "C" {
#endif

void get_differential_drive(
    void *src, void *dest);

void get_differential_drive_config(
    void *src, void *dest);

void get_differential_drive_sensor(
    void *src, void *dest);

void set_differential_drive_actuation(
    void *src, void *dest);

#ifdef __cplusplus
}
#endif

#endif
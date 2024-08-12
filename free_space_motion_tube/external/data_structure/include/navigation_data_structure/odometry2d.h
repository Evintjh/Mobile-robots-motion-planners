#ifndef NAVIGATION_DATA_STRUCTURE_ODOMETRY2D_H
#define NAVIGATION_DATA_STRUCTURE_ODOMETRY2D_H

#include <geometry_data_structure/pose2d.h>
#include <mechanics_data_structure/velocity2d.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct odometry2d_s{
    pose2d_t pose;
    velocity2d_t velocity;
}odometry2d_t;

#ifdef __cplusplus
}
#endif

#endif
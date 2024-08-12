#ifndef PLATFORM_DATA_STRUCTURE_DELTA_TRICYCLE_H
#define PLATFORM_DATA_STRUCTURE_DELTA_TRICYCLE_H

#include <geometry_data_structure/pose2d.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct delta_tricycle_s{
    struct wheel{
        void *drive;
        pose2d_t pose;  ///< pose of the wheel w.r.t. center of rear axle
    }wheel;
    double width;  ///< Width of the platform (m)
    double length;  ///< Length of the platform (m)
}delta_tricycle_t;

#ifdef __cplusplus
}
#endif

#endif
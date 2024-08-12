#ifndef MOTION_PRIMITIVE_STRUCTURE_MANEUVER_H
#define MOTION_PRIMITIVE_STRUCTURE_MANEUVER_H

#include<kinematics_data_structure/models.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct maneuver_s{
    double time_horizon;
    void *control;
}maneuver_t;

#ifdef __cplusplus
}
#endif

#endif
#ifndef KINEMATICS_DATA_STRUCTURE_CONTROL_H
#define KINEMATICS_DATA_STRUCTURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct unicycle_control_s{
    double forward_velocity;
    double angular_rate;
}unicycle_control_t;

#ifdef __cplusplus
}
#endif

#endif
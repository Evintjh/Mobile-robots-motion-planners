
#ifndef ACTUACTION_DATA_STRUCTURE_DIFFERENTIAL_DRIVE_ACTUATION_H
#define ACTUACTION_DATA_STRUCTURE_DIFFERENTIAL_DRIVE_ACTUATION_H

#include <time.h>

#define DIFFERENTIAL_DRIVE_COMMAND_MODE_TORQUE	  (0x0 << 2)
#define DIFFERENTIAL_DRIVE_COMMAND_MODE_VELOCITY	  (0x2 << 2)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct differential_drive_actuation_s{
    struct timespec timestamp;    
    int mode;
    struct velocity{
        double left_wheel, right_wheel;
    }velocity;
    struct current{
        double left_wheel, right_wheel;
    }current;
}differential_drive_actuation_t;

#ifdef __cplusplus
}
#endif

#endif
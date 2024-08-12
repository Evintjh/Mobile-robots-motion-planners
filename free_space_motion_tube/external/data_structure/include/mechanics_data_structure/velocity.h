#ifndef VELOCITY_DATA_STRUCTURE_H
#define VELOCITY_DATA_STRUCTURE_H

typedef struct velocity_s{
    double vx, vy, vz;  ///< linear velocity (m/s)
    double wx, wy, wz;  ///< angular velocity (rad/s)
}velocity_t;

#endif
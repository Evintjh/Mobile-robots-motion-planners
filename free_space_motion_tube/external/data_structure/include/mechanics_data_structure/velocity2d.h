#ifndef MECHANICS_DATA_STRUCTUREVELOCITY2D_H
#define MECHANICS_DATA_STRUCTUREVELOCITY2D_H

typedef struct velocity2d_s{
    double vx, vy;  ///< linear velocity (m/s)
    double w;  ///< angular velocity (rad/s)
}velocity2d_t;

#endif
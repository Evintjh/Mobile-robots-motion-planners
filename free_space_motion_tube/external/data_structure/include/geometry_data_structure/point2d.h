#ifndef POINT2D_H
#define POINT2D_H

#include <stdint.h>

typedef struct point2d_s{
    double x, y;
}point2d_t;

typedef struct point2d_array_s{
    point2d_t *points;
    int number_of_points;
    int max_number_of_points;        
}point2d_array_t;

#endif

#ifndef POLYLINE_H
#define POLYLINE_H

#include <stdint.h>
#include <geometry_data_structure/point2d.h>

typedef struct polyline_s{
    point2d_t *points;
    int number_of_points;
    int max_number_of_points;
}polyline_t;

#endif


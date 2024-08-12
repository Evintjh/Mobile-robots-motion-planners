#ifndef NAVIGATION_DATA_STRUCTURE_RANGE_MOTION_TUBE_H
#define NAVIGATION_DATA_STRUCTURE_RANGE_MOTION_TUBE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct range_motion_tube_s{
    int number_of_elements;
    double *range;
    double *angle;
    int *index;
    bool available;
}range_motion_tube_t;

#ifdef __cplusplus
}
#endif

#endif

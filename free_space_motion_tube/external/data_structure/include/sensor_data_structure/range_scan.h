#ifndef RANGE_SCAN_DATA_STRUCTURE_H
#define RANGE_SCAN_DATA_STRUCTURE_H

#include <time.h>

typedef struct range_scan_s{
    struct timespec timestamp;    ///< As defined in time.h
    double *angles;  ///< angle[i] is the direction of the i-th sensor beam (radians)
    double *measurements;   ///< measurement[i] is the distance to the object that reflected the i-th sensor beam (meters) 
    int nb_measurements;   ///< number of measurements in the current scan reading
}range_scan_t;

#endif
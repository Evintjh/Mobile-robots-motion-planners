#ifndef CORNER_H
#define CORNER_H

#include <geometry_data_structure/line_segment.h>
#include <geometry_data_structure/point.h>

#define NO_CORNER 0   
#define CONVEX_CORNER 1
#define CONCAVE_CORNER 2
#define OPEN_CORNER 3  // One of the lines is not visible

typedef struct corner_s{
    int corner_type;  ///< Type of corner as described above
    point_t coordinates;  ///< Coordinates of the corner
    line_segment_t *line_segments[2];  ///< Line segments joined by the corner
}corner_t; 

#endif
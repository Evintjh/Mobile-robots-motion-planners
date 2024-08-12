#ifndef LINE_SEGMENT2D_H
#define LINE_SEGMENT2D_H

#include <geometry_data_structure/point2d.h>

typedef struct line_segment2d_s{
    point2d_t endpoints[2];  ///< Pointer to points that define a line segment
}line_segment2d_t;

#endif

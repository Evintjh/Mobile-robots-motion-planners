#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H

#include <geometry_data_structure/point.h>

typedef struct line_segment_s{
    point_t endpoints[2];  ///< Two points that define a line segment
}line_segment_t;

#endif
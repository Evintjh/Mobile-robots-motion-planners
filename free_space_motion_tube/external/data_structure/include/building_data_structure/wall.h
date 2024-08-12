#ifndef WALL_H
#define WALL_H

#include <geometry_data_structure/line_segment.h>
#include <geometry_data_structure/corner.h>

typedef struct wall_s{
    int nb_line_segments;
    int nb_corners;
    line_segment_t *wall_segments;  ///< array of wall segments (not free space)
    corner_t *corners;  ///< array of corners in a wall (junctions, doors, free space)
}wall_t; 

#endif
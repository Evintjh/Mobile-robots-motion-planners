#ifndef CORRIDOR_H
#define CORRIDOR_H

#include <building_data_structure/wall.h>

typedef struct corridor_s{
    wall_t walls[2];
    double width, length;
    // add a pointer for a data structure of type object [@TODO define object]
}corridor_t; 

#endif
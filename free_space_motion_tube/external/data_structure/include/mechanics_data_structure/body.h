#ifndef BODY_H
#define BODY_H

#include <stdint.h>
#include <geometry_data_structure/polyline.h>

enum geometry{POLYLINE};

typedef struct body_s{
    enum geometry type;
    void *geometry;
}body_t;

#endif

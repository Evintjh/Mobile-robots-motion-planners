#ifndef KELO_TRICYCLE_H
#define KELO_TRICYCLE_H

#include <platform_data_structure/kelo_wheel.h>
#include <read_file/read_file.h>

typedef struct kelo_tricycle_s{
    kelo_wheel_t kelo_wheel;
    double wheelbase;  ///< distance between the centers of rear and front wheels (m)
    double width;  ///< Width of the platform (m)
    double length;  ///< Length of the platform (m)
}kelo_tricycle_t;

void configure_kelo_tricycle(const char *file_path, 
    kelo_tricycle_t *kelo_tricycle, int *status);

#endif
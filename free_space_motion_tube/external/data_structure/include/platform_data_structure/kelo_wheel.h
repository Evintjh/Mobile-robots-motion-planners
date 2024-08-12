#ifndef KELO_WHEEL_H
#define KELO_WHEEL_H

#include <stdint.h>
#include <read_file/read_file.h>

typedef struct kelo_wheel_s{
    double wheel_radius;  ///< radius of kelo wheels (assumed to be the same for both wheels)
    double wheel_track;  ///< distance between left and right wheels 
    double pivot_offset;  ///< distance from center of the wheels to center of rotation of the kelo base 
}kelo_wheel_t;

void configure_kelo_wheel(const char *file_path, 
    kelo_wheel_t *kelo_wheel, int *status);
#endif
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct encoder_s{
    int pulses;  ///< number of pulses detected
    double position;  ///< encoder position in rad (do not wrap) 
    double velocity;  ///< encoder velocity in rad/s
}encoder_t;

#endif
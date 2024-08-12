#ifndef KELO_INPUT_DATA_STRUCTURE_H
#define KELO_INPUT_DATA_STRUCTURE_H

#include <platform_data_structure/kelo_wheel.h>

#define KELO_COMMAND_MODE_TORQUE	  (0x0 << 2)
#define KELO_COMMAND_MODE_VELOCITY	  (0x2 << 2)

typedef struct kelo_input_s{
    uint16_t  command_mode;		// Command bits as defined in KELO_COMMAND_MODE
    float  left_wheel_setpoint;
    float  right_wheel_setpoint;
}kelo_input_t;

#endif
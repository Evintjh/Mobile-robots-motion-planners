#ifndef RANGE_SENSOR_DATA_STRUCTURE_H
#define RANGE_SENSOR_DATA_STRUCTURE_H

#include<model_data_structure/semantic_id.h>

typedef struct range_sensor_s{
    semantic_id_t semantic_id;
    double min_angle, max_angle;    ///< angular limits of the sensor [radians]
    double min_distance, max_distance;    ///< range distance limits [meters]
    double accuracy;    ///< accuracy of the sensor [%/meter]
    double angular_resolution;    ///< angular distance between two beams
    int nb_measurements;    ///< number of beams per scan reading
}range_sensor_t;

#endif

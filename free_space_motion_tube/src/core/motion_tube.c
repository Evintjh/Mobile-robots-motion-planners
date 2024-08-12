#include<free_space_motion_tube/core/motion_tube.h>
#include<free_space_motion_tube/core/motion_tube_cartesian.h>
#include<free_space_motion_tube/core/motion_tube_sensor_space.h>

#include<math.h>

#include <stdio.h>

const struct MotionTube MotionTube ={
    .create = motion_tube_create,
    .allocate_memory = motion_tube_allocate_memory,
    .deallocate_memory = motion_tube_deallocate_memory,
    .sample = motion_tube_sample,
    .Monitor.availability = motion_tube_availability
};

/** MotionTube methods **/

void motion_tube_create(motion_tube_t *motion_tube){
    MotionTubeCartesian.create(&motion_tube->cartesian);
    MotionTubeSensorSpace.create(&motion_tube->sensor_space);
}

void motion_tube_allocate_memory(motion_tube_t *motion_tube, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE){
    // Cartesian motion_tube
    MotionTubeCartesian.allocate_memory(&motion_tube->cartesian, max_number_of_samples, ALLOCATION_MODE);

    // Sensor space motion_tube
    size_t max_number_of_beams = 0;

    point2d_array_t *side[3] = {&motion_tube->cartesian.left, &motion_tube->cartesian.front, &motion_tube->cartesian.right};
    for(int i=0; i<3; i++){
        if (side[i]->points != NULL && side[i]->max_number_of_points > 0){
            max_number_of_beams += side[i]->max_number_of_points;
        }
    }
    MotionTubeSensorSpace.allocate_memory(&motion_tube->sensor_space, max_number_of_beams);

}

void motion_tube_deallocate_memory(motion_tube_t *motion_tube){
    MotionTubeCartesian.deallocate_memory(&motion_tube->cartesian);
    MotionTubeSensorSpace.deallocate_memory(&motion_tube->sensor_space);
}

void motion_tube_sample(motion_tube_t* motion_tube, double sampling_interval,
    const point2d_t *footprint,const motion_primitive_t* motion_primitive,
     const range_sensor_t* range_sensor, const pose2d_t* pose_sensor){
    // Cartesian
    MotionTubeCartesian.sample(&motion_tube->cartesian, sampling_interval, footprint, motion_primitive);
    // Sensor space
    motion_tube_cartesian_to_sensor_space(&motion_tube->cartesian,
        range_sensor, pose_sensor, &motion_tube->sensor_space);
}

void motion_tube_availability(const motion_tube_t* motion_tube, const lidar_t* lidar, bool* is_available){
    MotionTubeSensorSpace.Monitor.availability(&motion_tube->sensor_space, lidar, is_available);
}

/** Generic methods **/
void motion_tube_cartesian_to_sensor_space(const motion_tube_cartesian_t* motion_tube_cartesian,
    const range_sensor_t* range_sensor, const pose2d_t* pose_sensor,
    motion_tube_sensor_space_t* motion_tube_sensor_space){

    free_space_beam_t *beams = motion_tube_sensor_space->beams;
    int *number_of_beams = &motion_tube_sensor_space->number_of_beams;
    int max_number_of_beams = motion_tube_sensor_space->max_number_of_beams;
    
    const point2d_array_t *samples[3] = {&motion_tube_cartesian->left, 
        &motion_tube_cartesian->front, &motion_tube_cartesian->right};

    vector2d_t ray;
    point2d_t position_sensor = {.x=pose_sensor->x, .y =pose_sensor->y};
    *number_of_beams = 0; 
    motion_tube_sensor_space->fully_mapped_points_to_beams = true;
    for (int k=0; k<3; k++){
        for(int i=0; i<samples[k]->number_of_points; i++){
            if(*number_of_beams >= motion_tube_sensor_space->max_number_of_beams){
                break;
            }
            points_to_vector2d(&position_sensor, &samples[k]->points[i], &ray);
            if ( (ray.direction >= range_sensor->min_angle) && 
                 (ray.direction <= range_sensor->max_angle) &&
                 (ray.magnitude > range_sensor->min_distance) &&
                 (ray.magnitude < range_sensor->max_distance))
            {
                beams[*number_of_beams].type = FREE_SPACE;
                beams[*number_of_beams].range_outer = ray.magnitude;
                beams[*number_of_beams].angle = ray.direction;
                beams[*number_of_beams].index = round((ray.direction - 
                        range_sensor->min_angle)/range_sensor->angular_resolution);
                *number_of_beams += 1;
            }else
            {
                motion_tube_sensor_space->fully_mapped_points_to_beams = false;
            }
        }
    }  
}


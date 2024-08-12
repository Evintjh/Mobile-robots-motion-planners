#include <free_space_motion_tube/core/basic.h>
#include <math.h>

void points_to_vector2d(
    const point2d_t *origin,
    const point2d_t *end,
    vector2d_t *vector)
{
    vector->magnitude = sqrt(pow(end->x - origin->x,2) + 
        pow(end->y - origin->y,2) );
    vector->direction = atan2( end->y - origin->y, end->x - origin->x);
}

void sample_line_segment(const line_segment2d_t *line_segment, 
    double sampling_interval,
    point2d_array_t *samples)
{
    const point2d_t *p_init, *p_end;
    int number_of_samples;
    double length;
    point2d_t direction;
    
    p_init = &line_segment->endpoints[0];
    p_end = &line_segment->endpoints[1];
    
    length = sqrt(pow(p_init->x - p_end->x,2) + pow(p_init->y - p_end->y,2));
    direction.x = (p_end->x - p_init->x)/length;
    direction.y = (p_end->y - p_init->y)/length;
    number_of_samples = (int) ceil(length/sampling_interval) + 1;
    
    // Refine sample interval to contain end-points
    sampling_interval = length/(number_of_samples-1);

    for(int i=0; i<number_of_samples; i++){
        if (samples->number_of_points >= samples->max_number_of_points){   
            break;
        }
        samples->points[samples->number_of_points].x = p_init->x + i*sampling_interval*direction.x;
        samples->points[samples->number_of_points].y = p_init->y + i*sampling_interval*direction.y;
        samples->number_of_points += 1;
    }
}

void rigid_body_2d_transformation(const pose2d_t *pose, const point2d_t *p_reference,
    point2d_t *p_target){
    
    double cosz = cos(pose->yaw);
    double sinz = sin(pose->yaw);
    
    p_target->x = p_reference->x*cosz - p_reference->y*sinz + pose->x;
    p_target->y = p_reference->x*sinz + p_reference->y*cosz + pose->y;  
}

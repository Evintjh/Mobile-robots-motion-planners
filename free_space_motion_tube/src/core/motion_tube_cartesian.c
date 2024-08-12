#include<free_space_motion_tube/core/motion_tube_cartesian.h>
#include<math.h>
#include <stdio.h>

const struct MotionTubeCartesian MotionTubeCartesian ={
    .create = motion_tube_cartesian_create,
    .allocate_memory = motion_tube_cartesian_allocate_memory,
    .deallocate_memory = motion_tube_cartesian_deallocate_memory,
    .sample = motion_tube_cartesian_sample
};

void motion_tube_cartesian_create(motion_tube_cartesian_t *motion_tube){
    point2d_array_t *side[3] = {&motion_tube->left, &motion_tube->front, &motion_tube->right};
    for(int i=0; i<3; i++){
        side[i]->number_of_points = 0;
        side[i]->max_number_of_points = 0;
        side[i]->points = NULL;    
    }
}

void motion_tube_cartesian_allocate_memory(motion_tube_cartesian_t *motion_tube, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE){
    point2d_array_t *side[3] = {&motion_tube->left, &motion_tube->front, &motion_tube->right};

    if (ALLOCATION_MODE == 1){
        if (max_number_of_samples[0] > 0){
            for(int i=0; i<3; i++){ 
                side[i]->number_of_points = 0;
                side[i]->max_number_of_points = max_number_of_samples[0]; 
                side[i]->points = (point2d_t *) malloc(max_number_of_samples[0] * sizeof(point2d_t));  
                if (side[i]->points != NULL){
                    side[i]->max_number_of_points = max_number_of_samples[0];
                }else{
                    // Failed to allocate memory
                    side[i]->max_number_of_points = 0;
                }      
            } 
        }
    }
}

void motion_tube_cartesian_deallocate_memory(motion_tube_cartesian_t *motion_tube){
    point2d_array_t *side[3] = {&motion_tube->left, &motion_tube->front, &motion_tube->right};

    for(int i=0; i<3; i++){
        side[i]->number_of_points = 0;
        side[i]->max_number_of_points = 0;
        if (side[i]->points != NULL){
            free(side[i]->points);
            side[i]->points = NULL;        
        }    
    }
}

void motion_tube_cartesian_sample(motion_tube_cartesian_t *motion_tube, double sampling_interval,
    const point2d_t *footprint, const motion_primitive_t *motion_primitive)
{
    motion_tube->left.number_of_points = 0;
    motion_tube->front.number_of_points = 0;
    motion_tube->right.number_of_points = 0;

    if (motion_primitive->model == UNICYCLE){
        unicycle_control_t *control = (unicycle_control_t *) motion_primitive->control;
        if (fabs(control->angular_rate) < 1e-3){
            if (control->forward_velocity != 0){
                motion_tube_cartesian_sample_move_straight(motion_tube, sampling_interval,  
                    footprint, motion_primitive, &MotionPrimitiveUnicycle);
            }
        } else if (control->angular_rate > 0){
            motion_tube_cartesian_sample_steer_left(motion_tube, sampling_interval,
                footprint, motion_primitive, &MotionPrimitiveUnicycle);
        } else if (control->angular_rate < 0){
            motion_tube_cartesian_sample_steer_right(motion_tube, sampling_interval, 
                footprint, motion_primitive, &MotionPrimitiveUnicycle);
        }
    }
}

void motion_tube_cartesian_sample_move_straight(motion_tube_cartesian_t *motion_tube,
    double sampling_interval, const point2d_t* footprint, const motion_primitive_t *motion_primitive, 
    const struct MotionPrimitive *MotionPrimitive)
{
    unicycle_control_t *control = (unicycle_control_t*) motion_primitive->control;
    double inflation = 0.025*(control->forward_velocity/0.25-1);
    if(inflation <0)
    {
        inflation = 0;
    }
    point2d_t p_front_left = {
        .x = footprint[FRONT_LEFT].x,
        .y = footprint[FRONT_LEFT].y + inflation
    };
    point2d_t p_front_right = {
        .x = footprint[FRONT_RIGHT].x,
        .y = footprint[FRONT_RIGHT].y - inflation
    };

    // Left side
    MotionPrimitive->sample(motion_primitive, &p_front_left, 
        sampling_interval, &motion_tube->left );
    // Right side
    MotionPrimitive->sample(motion_primitive, &p_front_right, 
        sampling_interval, &motion_tube->right );
    // Front
    line_segment2d_t front = {
        .endpoints[0] = {
            .x = motion_tube->left.points[motion_tube->left.number_of_points-1].x + sampling_interval,
            .y = motion_tube->left.points[motion_tube->left.number_of_points-1].y},
        .endpoints[1] = {
            .x = motion_tube->right.points[motion_tube->right.number_of_points-1].x + sampling_interval,
            .y = motion_tube->right.points[motion_tube->right.number_of_points-1].y}
    };
    sample_line_segment(&front, sampling_interval, &motion_tube->front);
}

void motion_tube_cartesian_sample_steer_left(motion_tube_cartesian_t *motion_tube, 
    double sampling_interval, const point2d_t* footprint, const motion_primitive_t *motion_primitive, 
    const struct MotionPrimitive *MotionPrimitive)
{
    unicycle_control_t *control = (unicycle_control_t*) motion_primitive->control;
    double inflation = 0.025*(control->forward_velocity/0.25-1);
    if(inflation <0)
    {
        inflation = 0;
    }
    point2d_t p_front_left = {
        .x = footprint[FRONT_LEFT].x,
        .y = footprint[FRONT_LEFT].y + inflation
    };
    point2d_t p_front_right = {
        .x = footprint[FRONT_RIGHT].x,
        .y = footprint[FRONT_RIGHT].y - inflation
    };
    point2d_t p_axle_left = {
        .x = footprint[AXLE_LEFT].x,
        .y = footprint[AXLE_LEFT].y + inflation
    };   
    
    pose2d_t pose_final;
    MotionPrimitive->excite(motion_primitive->control, motion_primitive->time_horizon,
        NULL, &pose_final);
        
    // Position of point of interest for corresponding sensor pose
    point2d_t p_front_left_endtime, p_front_right_endtime;
    rigid_body_2d_transformation(&pose_final, &p_front_left,
            &p_front_left_endtime);
    rigid_body_2d_transformation(&pose_final, &p_front_right,
            &p_front_right_endtime);

    /* Left side of the template */
    MotionPrimitive->sample(motion_primitive, &p_axle_left, 
        sampling_interval, &motion_tube->left );

    line_segment2d_t left_side;
    left_side.endpoints[0] = motion_tube->left.points[motion_tube->left.number_of_points-1];
    left_side.endpoints[1] = p_front_left_endtime; 
    motion_tube->left.number_of_points--;
    sample_line_segment(&left_side, sampling_interval, &motion_tube->left);
    motion_tube->left.number_of_points--;

    /* Right side of the template */
    MotionPrimitive->sample(motion_primitive, &p_front_right, 
        sampling_interval, &motion_tube->right );
    motion_tube->right.number_of_points--;

    /* Front of the template */
    line_segment2d_t front;
    front.endpoints[0] = p_front_left_endtime;
    front.endpoints[1] = p_front_right_endtime;
    sample_line_segment(&front, sampling_interval, &motion_tube->front);
}

void motion_tube_cartesian_sample_steer_right(motion_tube_cartesian_t *motion_tube,
    double sampling_interval, const point2d_t* footprint, const motion_primitive_t *motion_primitive, 
    const struct MotionPrimitive *MotionPrimitive)
{
    unicycle_control_t *control = (unicycle_control_t*) motion_primitive->control;
    double inflation = 0.025*(control->forward_velocity/0.25-1);
    if(inflation <0)
    {
        inflation = 0;
    }
    point2d_t p_front_left = {
        .x = footprint[FRONT_LEFT].x,
        .y = footprint[FRONT_LEFT].y +inflation
    };
    point2d_t p_front_right = {
        .x = footprint[FRONT_RIGHT].x,
        .y = footprint[FRONT_RIGHT].y -inflation
    };
    point2d_t p_axle_right = {
        .x = footprint[AXLE_RIGHT].x,
        .y = footprint[AXLE_RIGHT].y -inflation
    };

    pose2d_t pose;
    MotionPrimitive->excite(motion_primitive->control, motion_primitive->time_horizon,
        NULL, &pose);
        
    // Position of point of interest for corresponding platform pose
    point2d_t p_front_left_endtime, p_front_right_endtime;
    rigid_body_2d_transformation(&pose, &p_front_left,
            &p_front_left_endtime);
    rigid_body_2d_transformation(&pose, &p_front_right,
            &p_front_right_endtime);

    /* Left side of the template */
    MotionPrimitive->sample(motion_primitive, &p_front_left, 
        sampling_interval, &motion_tube->left );
    motion_tube->left.number_of_points--;

    /* Right side of the template */
    MotionPrimitive->sample(motion_primitive, &p_axle_right, 
        sampling_interval, &motion_tube->right );

    line_segment2d_t right_side;
    right_side.endpoints[0] = motion_tube->right.points[motion_tube->right.number_of_points-1];
    right_side.endpoints[1] = p_front_right_endtime; 
    motion_tube->right.number_of_points--;
    sample_line_segment(&right_side, sampling_interval, &motion_tube->right);
    motion_tube->right.number_of_points--;

    /* Front of the template */
    line_segment2d_t front;
    front.endpoints[0] = p_front_left_endtime;
    front.endpoints[1] = p_front_right_endtime;
    sample_line_segment(&front, sampling_interval, &motion_tube->front);
}

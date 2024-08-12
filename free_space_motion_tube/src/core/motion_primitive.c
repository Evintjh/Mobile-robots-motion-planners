#include <free_space_motion_tube/core/motion_primitive.h>
#include <math.h>
#include <stdlib.h>

const struct MotionPrimitive MotionPrimitiveUnicycle={
    .create = motion_primitive_unicycle_create,
    .allocate_memory = motion_primitive_unicycle_allocate_memory,
    .deallocate_memory = motion_primitive_unicycle_deallocate_memory,
    
    .sample = motion_primitive_unicycle_sample,
    .excite = motion_primitive_unicycle_excite
};

void motion_primitive_unicycle_create(motion_primitive_t* motion_primitive){
    motion_primitive->model = UNICYCLE;
    motion_primitive->time_horizon = 0;
    motion_primitive->control = NULL;
}

void motion_primitive_unicycle_allocate_memory(motion_primitive_t* motion_primitive){
    motion_primitive->control = (unicycle_control_t *) malloc(sizeof(unicycle_control_t));
    if (motion_primitive->control != NULL){
        unicycle_control_t *unicycle_control = (unicycle_control_t *) motion_primitive->control;
        unicycle_control->angular_rate = 0;
        unicycle_control->forward_velocity = 0;
    }
}

void motion_primitive_unicycle_deallocate_memory(motion_primitive_t* motion_primitive){
    if (motion_primitive->control != NULL){
        free(motion_primitive->control);
    }
    motion_primitive->control = NULL;
}

void motion_primitive_unicycle_sample(const motion_primitive_t *motion_primitive, 
    const point2d_t *offset, double sampling_interval, point2d_array_t *samples)
{

    // Variables defined to keep notation compact
    unicycle_control_t *control = (unicycle_control_t *) motion_primitive->control;
    
    // Variables computed by this function
    double sampling_time;
    int number_of_samples;

    if (fabs(control->angular_rate) < 1e-3){    
        if (control->forward_velocity == 0){
            number_of_samples = 0;
            return;
        }else{
            sampling_time = sampling_interval/fabs(control->forward_velocity);
        }
    }else{
        double radius_at_origin = control->forward_velocity/control->angular_rate;
        double radius_at_point = sqrt(pow(offset->x,2) + 
            pow(radius_at_origin - offset->y,2) );
        sampling_time = (sampling_interval/(fabs(control->angular_rate)*radius_at_point));
    }

    // Refine sampling time such that samples at t=0 and t=time_horizon are included
    if (motion_primitive->time_horizon > 0){    // Only predict trajectory in the  future
        number_of_samples = (int) ceil(motion_primitive->time_horizon/sampling_time) + 1; 
        sampling_time = motion_primitive->time_horizon/(number_of_samples - 1); 
    }else{
        number_of_samples = 0;
    }
    pose2d_t pose;
    for(int i=0; i<number_of_samples; i++){
        if (samples->number_of_points >= samples->max_number_of_points){   
            break;
        }
        // Pose of the unicycle model for control input and time
        motion_primitive_unicycle_excite(motion_primitive->control, i*sampling_time, NULL, &pose);
        // Position of point of interest for corresponding platform pose
        rigid_body_2d_transformation(&pose, offset,
            &samples->points[samples->number_of_points]);
        samples->number_of_points += 1;
    }
}

void motion_primitive_unicycle_excite(const void *control, 
    double time, const pose2d_t *pose_init, pose2d_t *pose_final)
{
    unicycle_control_t *unicycle_control = (unicycle_control_t *) control;

    double x0=0.0;
    double y0=0.0;
    double yaw0=0.0;
    
    if (pose_init!=NULL){
        yaw0 = pose_init->yaw;
        x0 = pose_init->x;
        y0 = pose_init->y;
    }
    pose_final->yaw = unicycle_control->angular_rate*time + yaw0; 
    double sinz = sin(pose_final->yaw);     
    double cosz = cos(pose_final->yaw);

    if (fabs(unicycle_control->angular_rate) <= 1e-3){
        pose_final->x = x0*cosz - y0*sinz + unicycle_control->forward_velocity*time;
        pose_final->y = x0*sinz + y0*cosz + unicycle_control->forward_velocity*time*sinz;
    }else{
        double radius = unicycle_control->forward_velocity/unicycle_control->angular_rate;
        pose_final->x = x0*cosz - y0*sinz + radius*sinz;
        pose_final->y = x0*sinz + y0*cosz + radius*(1-cosz);
    }
}



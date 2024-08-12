/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors:
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file free_space_motion_tube_activity.c
 * @date March 22, 2022
 **/

#include <math.h>
#include <free_space_motion_tube/activity/activity.h>
#include <time.h>

#include <free_space_motion_tube/core/motion_tube_cartesian.h>

/**
 * The config() has to be scheduled everytime a change in the LCSM occurs,
 * so it properly configures the schedule for the next iteration according
 * to the LCSM state, resources, task, ..
 * @param[in] activity data structure for the free space activity
 */
void free_space_activity_config(activity_t *activity)
{
    // Remove config() from the eventloop schedule in the next iteration
    remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
    // Deciding which schedule to add
    switch (activity->lcsm.state)
    {
    case CREATION:
        add_schedule_to_eventloop(&activity->schedule_table, "creation");
        break;
    case RESOURCE_CONFIGURATION:
        add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
        break;
    case PAUSING:
        add_schedule_to_eventloop(&activity->schedule_table, "pausing");
        break;
    case RUNNING:
        printf("[Free-space Motion Tube] Ready!\n");
        add_schedule_to_eventloop(&activity->schedule_table, "running");
        break;
    case CLEANING:
        break;
    case DONE:
        break;
    }
};

// Creation
void free_space_activity_creation_coordinate(activity_t *activity)
{
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;
    // Coordinating own activity
    if (activity->state.lcsm_flags.creation_complete)
        activity->lcsm.state = RESOURCE_CONFIGURATION;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void free_space_activity_creation_configure(activity_t *activity)
{
    if (activity->lcsm.state != CREATION)
    {
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "creation");
    }
}

void free_space_activity_creation_compute(activity_t *activity)
{
    // Set the flag below to true when the creation behaviour has finished
    free_space_activity_params_t *params = (free_space_activity_params_t *)activity->conf.params;
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *)activity->state.computational_state.continuous;

    activity->state.lcsm_flags.creation_complete = false;
    params->odometry.velocity.vx = 0;
    params->odometry.velocity.w = 0;
    params->desired_velocity.vx = 0;
    params->desired_velocity.wz = 0;
 
    if (*coord_state->lidar_ready)
    {
        params->range_sensor = *(params->rt_range_sensor);
        params->range_scan.measurements = malloc(sizeof *params->range_scan.measurements *
                                                 params->range_sensor.nb_measurements);
        params->range_scan.angles = malloc(sizeof *params->range_scan.angles *
                                           params->range_sensor.nb_measurements);
        continuous_state->range_motion_tube.angle = (double *)malloc(300 * sizeof(double));
        continuous_state->range_motion_tube.range = (double *)malloc(300 * sizeof(double));
        continuous_state->range_motion_tube.index = (int *)malloc(300 * sizeof(int));
        continuous_state->range_motion_tube.number_of_elements = 0;
        params->rt_range_motion_tube->angle = (double *)malloc(300 * sizeof(double));
        params->rt_range_motion_tube->range = (double *)malloc(300 * sizeof(double));
        params->rt_range_motion_tube->index = (int *)malloc(300 * sizeof(int));
        params->rt_range_motion_tube->number_of_elements = 0;
        // Check whether memory has been properly allocated
        if (params->range_scan.measurements != NULL && params->range_scan.angles != NULL)
            activity->state.lcsm_flags.creation_complete = true;
    }
}

void free_space_activity_creation(activity_t *activity)
{
    free_space_activity_creation_compute(activity);
    free_space_activity_creation_coordinate(activity);
    free_space_activity_creation_configure(activity);
}

// Resource configuration
void free_space_activity_resource_configuration_coordinate(activity_t *activity)
{
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;
    // Internal coordination
    if (activity->state.lcsm_flags.resource_configuration_complete)
    {
        switch (activity->state.lcsm_protocol)
        {
        case INITIALISATION:
            activity->lcsm.state = PAUSING;
            break;
        case EXECUTION:
            activity->lcsm.state = RUNNING;
            break;
        case DEINITIALISATION:
            activity->lcsm.state = DONE;
            activity->state.lcsm_flags.deletion_complete = true;
            break;
        }
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void free_space_activity_resource_configuration_configure(activity_t *activity)
{
    if (activity->lcsm.state != RESOURCE_CONFIGURATION)
    {
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
        // Update flags for next visit to the resource configuration LCS
        activity->state.lcsm_flags.resource_configuration_complete = false;
    }
}

void free_space_activity_resource_configuration_compute(activity_t *activity)
{
    free_space_activity_params_t *params = (free_space_activity_params_t *)activity->conf.params;
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *)activity->state.computational_state.continuous;

    // Read file
    int config_status_free_space_activity;
    configure_free_space_activity_from_file(params->configuration_file,
                                            params, &config_status_free_space_activity);
    
    // Memory allocation
    size_t max_number_of_samples[1] = {params->max_number_of_samples};

    // Building templates
    double forward_vel[6] = {0.05, .15, 0.2, 0.25, 0.3, 0.4};
    double angular_rate[21] = {-50, -40, -30, -25, -20, -15, -10, -9, -5, -3, 0, 3, 5, 9, 10, 15, 20, 25, 30, 40, 50};    
    for(int k=0; k<3; k++){
        for(int i=0; i<6; i++){
            for(int j=0; j<21; j++){
                // Memory allocation
                MotionTube.allocate_memory(&params->motion_tube[k][i][j], max_number_of_samples, 1);
                MotionPrimitiveUnicycle.allocate_memory(&params->motion_primitive[k][i][j]);
                // Sample motion tube (cartesian & sensor space)    
                params->motion_primitive[k][i][j].time_horizon = params->time_horizon[k];
                ((unicycle_control_t *) params->motion_primitive[k][i][j].control)->forward_velocity = forward_vel[i];
                ((unicycle_control_t *) params->motion_primitive[k][i][j].control)->angular_rate = angular_rate[j]*(M_PI/180);
                MotionTube.sample(&params->motion_tube[k][i][j], params->sampling_interval[k], 
                    params->footprint, &params->motion_primitive[k][i][j], &params->range_sensor, &params->pose_sensor);
            }
        }
    }

    // Set the flag below to true when the resource configuartion behaviour has finished
    activity->state.lcsm_flags.resource_configuration_complete = true;
}

void free_space_activity_resource_configuration(activity_t *activity)
{
    free_space_activity_resource_configuration_compute(activity);
    free_space_activity_resource_configuration_coordinate(activity);
    free_space_activity_resource_configuration_configure(activity);
}

// Pausing
void free_space_activity_pausing_coordinate(activity_t *activity)
{
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;
    // Coordinating with other activities
    if (*coord_state->lidar_ready)
        activity->state.lcsm_protocol = EXECUTION;
    if (*coord_state->lidar_dead)
        activity->state.lcsm_protocol = DEINITIALISATION;

    // Coordinating own activity
    switch (activity->state.lcsm_protocol)
    {
    case EXECUTION:
        activity->lcsm.state = RUNNING;
        break;
    case DEINITIALISATION:
        activity->lcsm.state = RESOURCE_CONFIGURATION;
        break;
    }
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void free_space_activity_pausing_configure(activity_t *activity)
{
    if (activity->lcsm.state != PAUSING)
    {
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "pausing");
    }
}

void free_space_activity_pausing(activity_t *activity)
{
    free_space_activity_pausing_coordinate(activity);
    free_space_activity_pausing_configure(activity);
}

// Running
void free_space_activity_running_communicate_sensor_and_estimation(activity_t *activity)
{
    free_space_activity_params_t *params = (free_space_activity_params_t *)activity->conf.params;
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *)activity->state.computational_state.continuous;
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;

    // Copying range measurements from shared memory to a local buffer
    if(coord_state->range_scan_lock != NULL){
        pthread_mutex_lock(coord_state->range_scan_lock);
        params->range_scan.nb_measurements = params->rt_range_scan->nb_measurements;
        for (int i = 0; i < params->range_scan.nb_measurements; i++)
        {
            params->range_scan.measurements[i] = params->rt_range_scan->measurements[i];
            params->range_scan.angles[i] = params->rt_range_scan->angles[i];
        }
        pthread_mutex_unlock(coord_state->range_scan_lock);
    }

    // Copying velocity from shared memory to a local buffer
    if (coord_state->odometry_lock != NULL && params->rt_odometry!= NULL ){
        pthread_mutex_lock(coord_state->odometry_lock);
        params->odometry.pose = params->rt_odometry->pose;
        params->odometry.velocity = params->rt_odometry->velocity;
        pthread_mutex_unlock(coord_state->odometry_lock);
    }

    // Copying velocity setpoint
    if (coord_state->desired_velocity_lock != NULL && params->rt_desired_velocity!= NULL ){
        pthread_mutex_lock(coord_state->desired_velocity_lock);
        params->desired_velocity = *params->rt_desired_velocity;
        pthread_mutex_unlock(coord_state->desired_velocity_lock);
    }
}

void free_space_activity_running_coordinate(activity_t *activity)
{
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;
    // Coordinating with other activities
    if (*coord_state->deinitialisation_request ||
        *coord_state->lidar_dead)
        activity->state.lcsm_protocol = DEINITIALISATION;

    switch (activity->state.lcsm_protocol)
    {
    case DEINITIALISATION:
        activity->lcsm.state = RESOURCE_CONFIGURATION;
        break;
    }
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void free_space_activity_running_configure(activity_t *activity)
{
    free_space_activity_params_t *params = (free_space_activity_params_t *)activity->conf.params;
    if (activity->lcsm.state != RUNNING)
    {
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "running");
    }
}

void free_space_activity_running_compute(activity_t *activity)
{
    free_space_activity_params_t *params = (free_space_activity_params_t *)activity->conf.params;
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *)activity->state.computational_state.continuous;
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;

    // Data available (see communication_sensor_and_estimation)
    range_scan_t *range_scan = &params->range_scan; // lastest measurements made available by the lidar activity
    range_sensor_t *range_sensor = &params->range_sensor; // parameters of the range sensor
    lidar_t lidar = {.range_scan = range_scan, .range_sensor = range_sensor};

    velocity2d_t *current_velocity = &params->odometry.velocity;
    velocity_t *desired_velocity = &params->desired_velocity;
    velocity_t *command_velocity = &params->command_velocity;
    if(desired_velocity->vx <0.01 && fabs(desired_velocity->wz) < 0.01){
	command_velocity->vx = desired_velocity->vx;
        command_velocity->wz = 0;
	return;	
    }

    // // Motion tube
    double forward_vel[6] = {0.05, .15, 0.2, 0.25, 0.3, 0.4};
    double angular_rate[21] = {-50, -40, -30, -25, -20, -15, -10, -9, -5, -2, 0, 2, 5, 9, 10, 15, 20, 25, 30, 40, 50};

    double max_forward_vel_step = 0.2;
    double max_angular_rate_step = 0.4;
    uint8_t velocity_availability[6][21] = {0};

    for(int i=0; i<6; i++){
        if (fabs(current_velocity->vx - forward_vel[i]) > max_forward_vel_step){
            continue;
        }
        for(int j=0; j<21; j++){
            if(fabs(current_velocity->w - angular_rate[j]*(M_PI/180) ) > max_angular_rate_step){
                continue;
            }
            for(int k=0; k<3; k++){  
                bool is_available = false;             
                MotionTube.Monitor.availability(&params->motion_tube[k][i][j], &lidar, &is_available);
                if(is_available){
                    velocity_availability[i][j] += 1;
                }else{
                    break;
                }
            }
        }
    }

    struct{
        int horizon;
        double forward_vel, error_forward_vel;
        double angular_rate, error_angular_rate;
        int i, j;
    }solution;
    solution.horizon = 0;
    solution.forward_vel = 0;//(odometry->vx - 0);
    solution.angular_rate = 0;//(odometry->wz - 0);
    solution.error_angular_rate = INFINITY;
    solution.error_forward_vel = INFINITY;

    for(int i=0; i<6; i++){
        for(int j=0; j<21; j++){
            if(velocity_availability[i][j] > 0 && velocity_availability[i][j] >= solution.horizon){
                double curr_forward_vel_error = fabs(forward_vel[i] - desired_velocity->vx);
                double curr_angular_rate_error = fabs(angular_rate[j]*(M_PI/180) - desired_velocity->wz);
                if(curr_forward_vel_error < solution.error_forward_vel || 
                    (curr_forward_vel_error == solution.error_forward_vel && 
                    curr_angular_rate_error < solution.error_angular_rate) )
                {
                    solution.horizon = velocity_availability[i][j]; 
                    solution.forward_vel = forward_vel[i];
                    solution.angular_rate = angular_rate[j]*(M_PI/180);
                    solution.error_forward_vel = curr_forward_vel_error;
                    solution.error_angular_rate = curr_angular_rate_error;
                    solution.i = i;
                    solution.j = j;
                }                
            }
        }
    }

    command_velocity->vx = solution.forward_vel;
    command_velocity->wz = solution.angular_rate;
    if (fabs(command_velocity->wz) > 0 && command_velocity->vx == 0)
	    command_velocity->vx = 0.01;
    //printf("command_velocity.vx: %.3f, command_velocity.wz: %.3f\n", 
    //    command_velocity->vx, command_velocity->wz);
    //printf("odometry.vx: %.3f, odometry->wz: %.3f\n", 
    //    current_velocity->vx, current_velocity->w);
    // // Copying to range_motion_tube
    range_motion_tube_t *range_motion_tube = &continuous_state->range_motion_tube;
    if(solution.horizon > 0){
         int i = solution.i;
         int j = solution.j;
         int k = solution.horizon-1;
         //printf("[%d][%d][%d] number of beams: %d\n",k, i, j, params->motion_tube[k][i][j].sensor_space.number_of_beams);
         for (int p = 0; p < params->motion_tube[k][i][j].sensor_space.number_of_beams; p++)
         {
             range_motion_tube->angle[p] = params->motion_tube[k][i][j].sensor_space.beams[p].angle;
             range_motion_tube->range[p] = params->motion_tube[k][i][j].sensor_space.beams[p].range_outer;
             range_motion_tube->index[p] = params->motion_tube[k][i][j].sensor_space.beams[p].index;
         }
         range_motion_tube->number_of_elements = params->motion_tube[k][i][j].sensor_space.number_of_beams;
         range_motion_tube->available = true;
    }else{
         range_motion_tube->number_of_elements = 0;
         range_motion_tube->available = false;
    }   
}

void free_space_activity_running_communicate_control(activity_t *activity)
{
    free_space_activity_params_t *params = (free_space_activity_params_t *)activity->conf.params;
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *)activity->state.computational_state.continuous;
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *)activity->state.coordination_state;

    range_motion_tube_t *range_motion_tube = &continuous_state->range_motion_tube;
    range_motion_tube_t *rt_range_motion_tube = params->rt_range_motion_tube;

    // Copying range measurements from shared memory to a local buffer
    if (coord_state->motion_tube_lock != NULL && params->rt_range_motion_tube!= NULL ){
        pthread_mutex_lock(coord_state->motion_tube_lock);
        for (int i = 0; i < range_motion_tube->number_of_elements; i++)
        {
            rt_range_motion_tube->angle[i] = range_motion_tube->angle[i];
            rt_range_motion_tube->range[i] = range_motion_tube->range[i];
            rt_range_motion_tube->index[i] = range_motion_tube->index[i];
        }
        rt_range_motion_tube->number_of_elements = range_motion_tube->number_of_elements;
        rt_range_motion_tube->available = range_motion_tube->available;
        pthread_mutex_unlock(coord_state->motion_tube_lock);
    }

    // Send velocity to the robot..
    velocity_t *rt_command_velocity = params->rt_command_velocity;
    velocity_t *command_velocity = &params->command_velocity;
    if (coord_state->platform_control_lock != NULL && params->rt_command_velocity!= NULL ){
        pthread_mutex_lock(coord_state->platform_control_lock);
        rt_command_velocity->vx = command_velocity->vx;
        rt_command_velocity->wz = command_velocity->wz;
        pthread_mutex_unlock(coord_state->platform_control_lock);
    }
}

void free_space_activity_running(activity_t *activity)
{
    free_space_activity_running_communicate_sensor_and_estimation(activity);
    free_space_activity_running_coordinate(activity);
    free_space_activity_running_configure(activity);
    free_space_activity_running_compute(activity);
    free_space_activity_running_communicate_control(activity);
}

// SCHEDULER
void free_space_activity_register_schedules(activity_t *activity)
{
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t)free_space_activity_config,
                      activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");

    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t)free_space_activity_creation,
                      activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

    schedule_t schedule_resource_configuration = {.number_of_functions = 0};
    register_function(&schedule_resource_configuration, (function_ptr_t)free_space_activity_resource_configuration,
                      activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_configuration,
                      "resource_configuration");

    schedule_t schedule_pausing = {.number_of_functions = 0};
    register_function(&schedule_pausing, (function_ptr_t)free_space_activity_pausing,
                      activity, "pausing");
    register_schedule(&activity->schedule_table, schedule_pausing,
                      "pausing");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t)free_space_activity_running,
                      activity, "running");
    register_schedule(&activity->schedule_table, schedule_running,
                      "running");
}

void free_space_activity_create_lcsm(activity_t *activity, const char *name_algorithm)
{
    activity->conf.params = malloc(sizeof(free_space_activity_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(free_space_activity_continuous_state_t));
    activity->state.computational_state.discrete = malloc(sizeof(free_space_activity_discrete_state_t));
    activity->state.coordination_state = malloc(sizeof(free_space_activity_coordination_state_t));

    free_space_activity_params_t *params = (free_space_activity_params_t *) activity->conf.params;
    for(int i=0; i<3; i++){
        for(int j=0; j<6; j++){
            for(int k=0; k<21; k++){
                MotionTube.create(&params->motion_tube[i][j][k]);
                MotionPrimitiveUnicycle.create(&params->motion_primitive[i][j][k]);
            }
        }
    }
    params->rt_odometry = NULL;
    params->rt_desired_velocity = NULL;
    params->rt_command_velocity = NULL;

    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;
    coord_state->range_scan_lock = NULL;
    coord_state->platform_control_lock = NULL;
    coord_state->odometry_lock = NULL;
    coord_state->desired_velocity_lock = NULL;
    coord_state->platform_control_lock = NULL;
    coord_state->motion_tube_lock = NULL;
}

void free_space_activity_resource_configure_lcsm(activity_t *activity)
{
    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;

    // Schedule table (adding config() for the first eventloop iteration)
    free_space_activity_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void free_space_activity_destroy_lcsm(activity_t *activity)
{
    destroy_activity(activity);
}

const free_space_activity_t ec_free_space_activity = {
    .create_lcsm = free_space_activity_create_lcsm,
    .resource_configure_lcsm = free_space_activity_resource_configure_lcsm,
    .destroy_lcsm = free_space_activity_destroy_lcsm,
};

// Configuration from file
void configure_free_space_activity_from_file(const char *file_path,
                                             free_space_activity_params_t *params, int *status)
{
    // define array
    int number_of_params = 0; // Amount of parameters to be read, set by user
    param_array_t param_array[20];

    int i = 0; // Keeps tracks of amount of elements inside param_array

    param_array[number_of_params] = (param_array_t){"range_sensor/pose/x", &(params->pose_sensor.x), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"range_sensor/pose/y", &(params->pose_sensor.y), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"range_sensor/pose/yaw", &(params->pose_sensor.yaw), PARAM_TYPE_DOUBLE};
    number_of_params++;

    param_array[number_of_params] = (param_array_t){"motion_tube/max_number_of_samples", &(params->max_number_of_samples), PARAM_TYPE_INT};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"motion_tube/sampling_interval/green", &(params->sampling_interval[2]), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"motion_tube/sampling_interval/yellow", &(params->sampling_interval[1]), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"motion_tube/sampling_interval/red", &(params->sampling_interval[0]), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"motion_tube/time_horizon/green", &(params->time_horizon[2]), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"motion_tube/time_horizon/yellow", &(params->time_horizon[1]), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"motion_tube/time_horizon/red", &(params->time_horizon[0]), PARAM_TYPE_DOUBLE};
    number_of_params++;

    param_array[number_of_params] = (param_array_t){"footprint/front/left/x", &(params->footprint[FRONT_LEFT].x), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"footprint/front/left/y", &(params->footprint[FRONT_LEFT].y), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"footprint/front/right/x", &(params->footprint[FRONT_RIGHT].x), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"footprint/front/right/y", &(params->footprint[FRONT_RIGHT].y), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"footprint/axle/left/x", &(params->footprint[AXLE_LEFT].x), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"footprint/axle/left/y", &(params->footprint[AXLE_LEFT].y), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"footprint/axle/right/x", &(params->footprint[AXLE_RIGHT].x), PARAM_TYPE_DOUBLE};
    number_of_params++;
    param_array[number_of_params] = (param_array_t){"footprint/axle/right/y", &(params->footprint[AXLE_RIGHT].y), PARAM_TYPE_DOUBLE};
    number_of_params++;
    // generic reader function
    int config_status_activity;

    // Activity itself
    read_from_input_file(file_path, param_array, number_of_params, &config_status_activity);

    // Other parameters

    // Verification
    if (config_status_activity == CONFIGURATION_FROM_FILE_SUCCEEDED)
    {
        *status = CONFIGURATION_FROM_FILE_SUCCEEDED;
    }
    else
    {
        *status = CONFIGURATION_FROM_FILE_FAILED;
    }
}

// Debugging prints!
/*
                    printf("best solution: ");
                    printf("(%d,%d): v:%f w:%f, horizon: %d\n", 
                        i,j,forward_vel[i], angular_rate[j], velocity_availability[i][j]);

    printf("----------------------------\n");

    struct timespec tstart={0,0}, tend={0,0};
    clock_gettime(CLOCK_MONOTONIC, &tstart);

    clock_gettime(CLOCK_MONOTONIC, &tend);
    double time = ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - 
           ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
    printf("%d tubes took about %.5f miliseconds\n", 3*3*5, time*1000);

    printf("maneuver. T: %f, maneuver.v: %.2f, maneuver.w: %.2f, is_available: %d\n", params->time_horizon[i],
        ((unicycle_control_t *)motion_primitive->control)->forward_velocity, ((unicycle_control_t *)motion_primitive->control)->angular_rate, is_available);

    printf("left has %d points:\n", motion_tube->cartesian.left.number_of_points);
    printf("front has %d points:\n", motion_tube->cartesian.front.number_of_points);
    printf("right has %d points:\n", motion_tube->cartesian.right.number_of_points);

    printf("maneuver. T: %f, maneuver.v: %.2f, maneuver.w: %.2f, is_available: %d\n", motion_primitive->time_horizon,
           ((unicycle_control_t *)motion_primitive->control)->forward_velocity, ((unicycle_control_t *)motion_primitive->control)->angular_rate, is_available);

    printf("left has %d points:\n", motion_tube->cartesian.left.number_of_points);
    for (int i = 0; i < motion_tube->cartesian.left.number_of_points; i++)
    {
        int j = i;
        printf("%d: (x,y) = (%f, %f)", i, motion_tube->cartesian.left.points[i].x, motion_tube->cartesian.left.points[i].y);
        printf(", range: %.2f, angle: %.2f, index: %d\n",
               motion_tube->sensor_space.beams[j].range_outer,
               motion_tube->sensor_space.beams[j].angle * 180 / M_PI,
               motion_tube->sensor_space.beams[j].index);
    }

    printf("front has %d points:\n", motion_tube->cartesian.front.number_of_points);
    for(int i=0; i<motion_tube->cartesian.front.number_of_points; i++){
        printf("%d: (x,y) = (%f, %f)", i, motion_tube->cartesian.front.points[i].x, motion_tube->cartesian.front.points[i].y);
        int j = i+motion_tube->cartesian.left.number_of_points;
        printf(", range: %.2f, angle: %.2f, index: %d\n",
             motion_tube->sensor_space.beams[j].range_outer,
             motion_tube->sensor_space.beams[j].angle*180/M_PI,
             motion_tube->sensor_space.beams[j].index);
    }

    printf("right has %d points:\n", motion_tube->cartesian.right.number_of_points);
    for(int i=0; i<motion_tube->cartesian.right.number_of_points; i++){
        printf("%d: (x,y) = (%f, %f)", i, motion_tube->cartesian.right.points[i].x, motion_tube->cartesian.right.points[i].y);
        int j = i+motion_tube->cartesian.front.number_of_points+motion_tube->cartesian.left.number_of_points;
        if (j+1 >  motion_tube->sensor_space.number_of_beams){
            printf("\n");
            continue;
        }
        printf(", range: %.2f, angle: %.2f, index: %d\n",
             motion_tube->sensor_space.beams[j].range_outer,
             motion_tube->sensor_space.beams[j].angle*180/M_PI,
             motion_tube->sensor_space.beams[j].index);
    }

    printf("--------------------\n");


*/

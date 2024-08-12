#include <five_c/thread/thread.h>
#include <five_c/activity/activity.h>

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <free_space_motion_tube/activity/activity.h>

bool platform_control_dead, lidar_dead;
bool deinitialisation_request = false;
static int interrupted;

static void sigint_handler(int sig) { 
  lidar_dead = true;
  platform_control_dead = true; 
}

int main(void) {
  signal(SIGINT, sigint_handler);
  thread_t main_thread;
  activity_t free_space_activity;
  // free_space_activity
  ec_free_space_activity.create_lcsm(&free_space_activity,
                                    "free_space_activity");
  ec_free_space_activity.resource_configure_lcsm(&free_space_activity);
  
  // Mocking activities
  bool platform_control_ready, lidar_ready;
  range_scan_t rt_range_scan={
    .nb_measurements = 627,
    .angles = (double*) malloc(sizeof(double)*627),
    .measurements = (double*) malloc(sizeof(double)*627),
  };
  range_sensor_t range_sensor={
    .angular_resolution = 0.006136,
    .max_angle = 1.920544,
    .min_angle = -1.920544,
    .max_distance = 5.5,
    .min_distance = .1,
    .nb_measurements = 627
  };
  for (int i=0; i< range_sensor.nb_measurements; i++){
    rt_range_scan.angles[i] = range_sensor.min_angle + i*
      range_sensor.angular_resolution;
    rt_range_scan.measurements[i] = 3.;
  }

  range_motion_tube_t range_motion_tube;
  odometry2d_t rt_odometry;
  kelo_tricycle_t platform;
  velocity_t rt_desired_velocity;
  pthread_mutex_t range_sensor_lock, platform_control_lock, odometry_lock;
  pthread_mutex_t motion_tube_lock;
  // Shared memory
  free_space_activity_coordination_state_t  *coord_state = 
  (free_space_activity_coordination_state_t *) free_space_activity.state.coordination_state; 
  free_space_activity_continuous_state_t  *continuous_state = 
  (free_space_activity_continuous_state_t *) free_space_activity.state.computational_state.continuous; 
  free_space_activity_params_t  *params = 
  (free_space_activity_params_t *) free_space_activity.conf.params; 

  coord_state->platform_control_ready = &platform_control_ready;
  coord_state->lidar_ready = &lidar_ready;
  coord_state->platform_control_dead = &platform_control_dead;
  coord_state->lidar_dead = &lidar_dead;
  coord_state->deinitialisation_request = &deinitialisation_request;
  coord_state->range_scan_lock = &range_sensor_lock;
  coord_state->odometry_lock = &odometry_lock;
  coord_state->platform_control_lock = &platform_control_lock;
  coord_state->motion_tube_lock = &motion_tube_lock;

  params->rt_desired_velocity = &rt_desired_velocity;
  params->rt_odometry = &rt_odometry;
  params->rt_range_scan = &rt_range_scan;
  params->rt_range_sensor = &range_sensor;
  params->rt_range_motion_tube = &range_motion_tube;

  strcpy(params->configuration_file, "../configuration/free_space.json");
  // Setting configuration file

  create_thread(&main_thread, "main_thread", 200);
  register_activity(&main_thread, &free_space_activity,
                    "free_space_activity");
  
  platform_control_ready = true;
  lidar_ready = true;

  do_thread_loop((void*) &main_thread);

  printf("Exiting...\n");
  return 0;
}

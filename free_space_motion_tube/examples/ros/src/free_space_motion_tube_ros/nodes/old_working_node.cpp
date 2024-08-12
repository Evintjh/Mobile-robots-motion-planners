// Standard
#include <sstream>

// ROS
#include "ros/ros.h"

// Messages
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

// Free space motion tube lib
#include "free_space_motion_tube/core/motion_tube.h"
#include "free_space_motion_tube/core/motion_primitive.h"

pose2d_t current_pose= {.x=0, .y=0, .yaw=0};
pose2d_t INIT_POSE = {.x = -2, .y = 3, .yaw = M_PI/2}; 
double des_angular_rate = 0;
double des_forward_speed = 0;
double cruise_velocity = 0.5;
double best_distance_to_goal = 10;
int success  = 0;
int index_offset = 0;
// GOAL_POSITION = [0, 10]  # relative to the initial position
ros::Publisher cmd_vel_pub;
bool has_goal = false;
point2d_t goal, closest_goal;

int count = 0;
void NavCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(++count < 0){
        ;//return;
    }
    else
    {
        count = 0;
    }
    // We acutally want a 1 m lookahead
    int index = 15;
    if (msg->poses.size() <= index)
    {
        index = msg->poses.size()-1;
    }
    point2d_t local_goal = 
        {
            .x = msg->poses[index].pose.position.x,
            .y = msg->poses[index].pose.position.y,
        };
    goal = local_goal;
    closest_goal.x = msg->poses[10].pose.position.x;
    closest_goal.y = msg->poses[10].pose.position.y;
    has_goal = true;
    std::cout <<  msg->poses.size() << std::endl;
}


void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!has_goal)
    {
        return;
    }
    range_sensor_t range_sensor;   
    range_sensor.angular_resolution = msg->angle_increment;
    range_sensor.nb_measurements = msg->ranges.size();
    range_sensor.max_angle = msg->angle_max;
    range_sensor.min_angle = msg->angle_min;
    range_sensor.max_distance = msg->range_max;
    range_sensor.min_distance = msg->range_min;

    range_scan_t range_scan;
    range_scan.nb_measurements = msg->ranges.size();
    range_scan.measurements = (double*) malloc(range_scan.nb_measurements*sizeof(double));
    range_scan.angles = (double*) malloc(range_scan.nb_measurements*sizeof(double));
    
    // Copying measurements
    for (int i=0; i< range_sensor.nb_measurements; i++)
    {
        range_scan.angles[i] = range_sensor.min_angle + i*
            range_sensor.angular_resolution;
        range_scan.measurements[i] = (double) msg->ranges[i];
    }

    pose2d_t pose_sensor = {.x = .055, .y =0, .yaw =0};
    point2d_t footprint[4];
    footprint[FRONT_LEFT].x = .31;
    footprint[FRONT_LEFT].y = 0.25;
    footprint[AXLE_LEFT].x =  -.31;
    footprint[AXLE_LEFT].y = 0.25;
    footprint[FRONT_RIGHT].x = .31;
    footprint[FRONT_RIGHT].y = -0.25;
    footprint[AXLE_RIGHT].x =  -.31;
    footprint[AXLE_RIGHT].y = -0.25;
    double time_horizon[] = {3};
    double sampling_interval[] = {0.05};
    size_t max_number_of_samples = 300;
    int nb_angular_rate = 91;
    int half_nb_angular_rate = (nb_angular_rate-1)/2;
    double MAX_ANGULAR_RATE = 2;

    if (success > 10)
    {
        time_horizon[0] = 1.5;
        cruise_velocity = .6;
    }else
    {
        time_horizon[0] = .5;
        cruise_velocity = .6;
        sampling_interval[0] = .05;

    }

    printf("here!\n");

    // Building templates
    double forward_vel[1] = {cruise_velocity};
    double angular_rate[nb_angular_rate]; 
    for(int i=-half_nb_angular_rate; i< half_nb_angular_rate+1;i++)
    {
        angular_rate[i+half_nb_angular_rate] = ((double) i)/half_nb_angular_rate * MAX_ANGULAR_RATE;
    }    
    // printf("here!\n");

    // Creating tubes
    motion_tube_t motion_tube[1][1][nb_angular_rate];
    motion_primitive_t motion_primitive[1][1][nb_angular_rate];
    for(int i=0; i<1; i++){
        for(int j=0; j<1; j++){
            for(int k=0; k<nb_angular_rate; k++){
                MotionTube.create(&motion_tube[i][j][k]);
                MotionPrimitiveUnicycle.create(&motion_primitive[i][j][k]);
            }
        }
    }
    // printf("here!\n");

    // configuring tubes
    for(int k=0; k<1; k++)
    {
        for(int i=0; i<1; i++)
        {
            for(int j=0; j<nb_angular_rate; j++)
            {
                // Memory allocation
                MotionTube.allocate_memory(&motion_tube[k][i][j], &max_number_of_samples, 1);
                MotionPrimitiveUnicycle.allocate_memory(&motion_primitive[k][i][j]);
                // Sample motion tube (cartesian & sensor space)    
                double local_time_horizon = std::min(M_PI/(2*fabs(angular_rate[j])+0.001),time_horizon[k] );
                motion_primitive[k][i][j].time_horizon = local_time_horizon;// time_horizon[k];
                ((unicycle_control_t *) motion_primitive[k][i][j].control)->forward_velocity = forward_vel[i];
                ((unicycle_control_t *) motion_primitive[k][i][j].control)->angular_rate = angular_rate[j];
                MotionTube.sample(&motion_tube[k][i][j], sampling_interval[k], 
                    footprint, &motion_primitive[k][i][j], &range_sensor, &pose_sensor);
                printf("%f, ", local_time_horizon);
            }
        }
    }
    printf("here!\n");

    // Evaluating tubes
    lidar_t lidar = 
    {
        .range_sensor = &range_sensor,
        .range_scan = &range_scan
    };
    bool at_least_one_tube_available = false;
    uint8_t velocity_availability[1][nb_angular_rate] = {0};
    for(int i=0; i<1; i++)
    {
        for(int j=0; j<nb_angular_rate; j++)
        {
             for(int k=0; k<1; k++)
             {  
                bool is_available = false;             
                MotionTube.Monitor.availability(&motion_tube[k][i][j], &lidar, &is_available);
                if(is_available){
                    velocity_availability[i][j] = 1;
                    at_least_one_tube_available = true;

                }else{
                    velocity_availability[i][j] = 0;
                }
            }
        }
    }

    // INIT_POSITION = [-2, 3, 1.57]  # in world frame
    // GOAL_POSITION = [0, 10]  # relative to the initial position
    double best_cost = 100;
    int best_cost_index;
    int largest_group_count = 0;
    int center_of_largest_group=0;
    int group_count = 0;

    pose2d_t copy_current_pose = current_pose;
    double c = cos(copy_current_pose.yaw), s = sin(copy_current_pose.yaw);
    bool group_of_best_index = false;
    point2d_t local_goal = goal;
    for(int i=0; i<1; i++)
    {
        for(int j=0; j<nb_angular_rate; j++)
        {
            for(int k=0; k<1; k++)
            {  
                if(velocity_availability[i][j])
                {
                    group_count += 1;
                    double v = forward_vel[i];
                    double w = angular_rate[j];
                    double T = time_horizon[k];
                    point2d_t estimated_position_body;
                    if (fabs(w) > 1e-3){
                        estimated_position_body.x = (v/w) * sin(T*w);
                        estimated_position_body.y = (v/w) * (1 - cos(T*w)); 
                    }else
                    {
                        estimated_position_body.x = v*T;
                        estimated_position_body.y = 0; 
                    }
                    point2d_t estimated_position_world = 
                    {
                        .x = c*estimated_position_body.x - s*estimated_position_body.y + copy_current_pose.x,
                        .y = s*estimated_position_body.x + c*estimated_position_body.y + copy_current_pose.y
                    };

                    double distance_to_goal = pow(estimated_position_world.x - local_goal.x, 2) +
                        pow(estimated_position_world.y - local_goal.y, 2);
                    double cost = distance_to_goal;
                    printf("x: %.2f, y: %.2f, cost: %f, vel: %f\n", 
                        estimated_position_world.x, estimated_position_world.y, cost, w );
                    if(cost < best_cost)
                    {
                        best_cost_index = j;
                        best_cost = cost;
                        best_distance_to_goal = sqrt(distance_to_goal);
                        group_of_best_index = true;
                    }
                }else
                {
                    if(group_count > largest_group_count && group_of_best_index)
                    {
                        largest_group_count = group_count;
                        center_of_largest_group = j - group_count/2;
                    }
                    group_count = 0;
                    group_of_best_index = false;
                }

            }
        }
    }
    
    if (group_count > 0)
    {
        if(group_count > largest_group_count && group_of_best_index)
        {
            largest_group_count = group_count;
            center_of_largest_group = (nb_angular_rate-1) - group_count/2;

        }        
    }
    std::cout << "best cost: " << best_cost << ", index: " << 
        best_cost_index << ", velocity: "  << angular_rate[best_cost_index] << std::endl;
    std::cout << "largest group count: " << largest_group_count << ", center: " << 
        center_of_largest_group << ", velocity: " << angular_rate[center_of_largest_group] << std::endl;

    int choosen_index;
    if (best_cost_index < center_of_largest_group + largest_group_count/2 - 3 &&
        best_cost_index > center_of_largest_group - largest_group_count/2 + 3)
    {
        choosen_index = best_cost_index;
        index_offset = 0;
    }else 
    {
        if (center_of_largest_group>best_cost_index)
            choosen_index = best_cost_index + 4;
        else
            choosen_index = best_cost_index - 4;
    }

    std::cout << "choosen index: " << choosen_index << std::endl << std::endl;

    if (largest_group_count > 0)
    {
        des_forward_speed = cruise_velocity;    
        des_angular_rate = angular_rate[choosen_index];
        if(largest_group_count > nb_angular_rate*.7)
        {
            if (fabs(des_angular_rate) > 1e-3)
            {
                double r = des_forward_speed/des_angular_rate;
                //des_forward_speed = .75;             
                //des_angular_rate = des_forward_speed/r;
            }
            //des_forward_speed = .75;             
        }

        success += 1;   
    }
    else
    {
        des_forward_speed *= 0;
        des_angular_rate =- .5*atan2(closest_goal.y - current_pose.y, 
            closest_goal.x - current_pose.x);
    }
    
    if (success > 10)
    {   
        success = 11;
    }
    else if (success < -10)
    {   
        success = -11;
    }


    for(int i=0; i<1; i++){
        for(int j=0; j<1; j++){
            for(int k=0; k<nb_angular_rate; k++){
                MotionTube.deallocate_memory(&motion_tube[i][j][k]);
                MotionPrimitiveUnicycle.deallocate_memory(&motion_primitive[i][j][k]);
            }
        }
    }
    printf("succes: %d\n", success);
    geometry_msgs::Twist Tmsg;

    double MAX_ANG_VEL = 0.75;
    if(fabs(des_angular_rate) > MAX_ANG_VEL)
    {
        double r = des_forward_speed/fabs(des_angular_rate);
        des_angular_rate = MAX_ANG_VEL*des_angular_rate/fabs(des_angular_rate);
        des_forward_speed = r*fabs(des_angular_rate);
    }
    Tmsg.linear.x = des_forward_speed;
    Tmsg.angular.z = des_angular_rate;
    cmd_vel_pub.publish(Tmsg);

    // Cleaning for next iteration
    if (range_scan.measurements){
        free(range_scan.measurements);
    }
    if (range_scan.angles)
    {
        free(range_scan.angles);
    }

}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    struct{
        double x;
        double y;
        double z;
        double w;
    }q;
    q.x = msg->pose.pose.orientation.x;
    q.y = msg->pose.pose.orientation.y;
    q.z = msg->pose.pose.orientation.z;
    q.w = msg->pose.pose.orientation.w;
    double yaw = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    // Transform to world frame
    //double c = cos(yaw), s= sin(yaw);
    current_pose.x = x;// + INIT_POSE.x; 
    current_pose.y = y;// + INIT_POSE.y;
    current_pose.yaw =  yaw;
    std::cout << current_pose.x << ", " << current_pose.y << ", " <<  current_pose.yaw  << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "free_space_motion_tube");
    ros::NodeHandle n;

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber scan_sub = n.subscribe("front/scan", 1000, ScanCallback);
    ros::Subscriber odom_sub = n.subscribe("odometry/filtered", 1000, OdomCallback);
    ros::Subscriber nav_sub = n.subscribe("/move_base/NavfnROS/plan", 100, NavCallback);
    std::cout << "READY"<< std::endl;

    ros::Rate loop_rate(10);

    ros::spin();


  return 0;
}
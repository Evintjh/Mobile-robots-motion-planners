// Standard
#include <sstream>
#include <time.h>
#include <algorithm>
#include <iostream>

// ROS
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>

// Messages
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// Motion tube libraries
#include "barn_motion_tube.h"

#define MAX_NUMBER_OF_WAYPOINTS 100
// #define LIDAR_OFFSET_X -0.055
// #define LIDAR_OFFSET_X -0.0
#define LIDAR_OFFSET_X -0.12

#define GLOBAL_PLANNER_WAYPOINT_SAMPLING_INTERVAL 0.025
#define LOOKAHEAD_LENGTH 0.065   // .1 to make sure is ahead

pose2d_t TransformPose(pose2d_t &in1, pose2d_t &in2);
double wrap_to_pi(double angle);
void PrintPoint2dArray(point2d_array_t *points);
double compute_points_distance(point2d_t *p1, point2d_t *p2);
int print = true;
double last_forward_vel = 0;
double acceleration = .5;
double dt = 1.0/50;
double last_w_vel=0.0;
double max_forward_vel = .5;
double max_angular_rate = M_PI/2;

bool new_plan_request = true;
class BarnMotionTubeNode
{
    private:
        // ROS
        ros::Publisher cmd_vel_pub_;
        ros::Publisher path_pub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber global_planner_sub_;
        ros::NodeHandle node_;

        // Input
        range_sensor_t range_sensor_;    //! sensor parameters
        range_scan_t range_scan_;        //! sensor measurements
        pose2d_t pose_odom_;             //! pose of the vehicle in odom frame
        pose2d_t pose_;                  //! pose of the vehicle in map frame
        pose2d_t vel_odom_;
        point2d_array_t waypoints_;
        // Motion tube
        BarnMotionTube barn_motion_tube_;
        // Goal
        double goal_distance_offset_;
        // Selection
        std::vector<std::vector<double>> cost_;

        typedef struct {
            double sim_period;
            double acc_lim_x;
            double acc_lim_y;
            double acc_lim_theta;
        } AccLimits;
        bool goal_arrived;
        double distance_to_goal;
        double goal_y_in_base;
        double goal_x_in_base;
        

    public:
        BarnMotionTubeNode(void);
        // Standard local base planner
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg); 
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void BrakeWithAccLimits();
        double sign(double x);
        // Alternative way to get nav_msgs
        void GlobalNavPlannerCallback(const nav_msgs::Path::ConstPtr& msg);
        geometry_msgs::PoseStamped goal_;

        
};

//! @brief Constructor sets up ROS publisher/subscriber and motion tubes
// configuration.
BarnMotionTubeNode::BarnMotionTubeNode(void)
{
    // Read parameter
    // @TODO: Change to a launch file
    // pose_of_odom_to_map_.x = -2;
    // pose_of_odom_to_map_.y = 3;
    // pose_of_odom_to_map_.yaw = 0;

    // Setting publisher and subscribers
    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_pub_ = node_.advertise<nav_msgs::Path>("projected_path", 1);
    scan_sub_ = node_.subscribe("laser_scan", 1000, 
        &BarnMotionTubeNode::ScanCallback, this);
    odom_sub_ = node_.subscribe("odom", 1000,  
        &BarnMotionTubeNode::OdomCallback, this);
    global_planner_sub_ = node_.subscribe("/move_base/NavfnROS/plan", 1000,  
        &BarnMotionTubeNode::GlobalNavPlannerCallback, this);
    goal_sub_ = node_.subscribe("/move_base_simple/goal", 1000, &BarnMotionTubeNode::GoalCallback, this);


    // Motion tubes
    range_sensor_t range_sensor; 
    // range_sensor.angular_resolution = 0.0065540750511;
    range_sensor.angular_resolution = 0.01745329238474369;
    // range_sensor.nb_measurements = 720;
    range_sensor.nb_measurements = 360;
    // range_sensor.max_angle = 2.3561899662;
    // range_sensor.min_angle = -2.3561899662;
    range_sensor.max_angle = 3.1241393089294434;
    range_sensor.min_angle = -3.1415927410125732;
    // range_sensor.max_distance = 30.0;
    // range_sensor.min_distance = 0.10;
    range_sensor.max_distance = 10.0;
    range_sensor.min_distance = 0.20;
    barn_motion_tube_.Configure(&range_sensor);

    // Allocate memory
    waypoints_.points = (point2d_t *) malloc(sizeof(point2d_t)*MAX_NUMBER_OF_WAYPOINTS);
    if(waypoints_.points==NULL)
    {
        waypoints_.max_number_of_points = 0;
    }else
    {
        waypoints_.max_number_of_points = MAX_NUMBER_OF_WAYPOINTS;
    }
    waypoints_.number_of_points = 0;
    // cost_ = (double **) malloc(sizeof(double*) * barn_motion_tube_.motion_tube_.size());
    // best_forward_vel_.resize(barn_motion_tube_.motion_tube_.size());
    // best_angular_vel_.resize(barn_motion_tube_.motion_tube_.size());
    // best_cost_.resize(barn_motion_tube_.motion_tube_.size());
    cost_.resize(barn_motion_tube_.motion_tube_.size());
    for(size_t i=0; i<barn_motion_tube_.motion_tube_.size();i++)
    {
        cost_[i].resize(barn_motion_tube_.motion_tube_[i].size());
        // cost_[i] = (double *) malloc(sizeof(double) * barn_motion_tube_.motion_tube_[i].size());
        // memset(cost_[i], 10000, sizeof(double) * barn_motion_tube_.motion_tube_[i].size());
    } 

    // Configuring internal variables -> lookahead must consider the footprint of the robot too
    goal_distance_offset_ = barn_motion_tube_.params_.footprint[FRONT_LEFT].x + LOOKAHEAD_LENGTH;
}

void BarnMotionTubeNode::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (waypoints_.number_of_points < 1){
        return;
    }

    // Copy lastest received odometry
    pose2d_t pose = pose_odom_;
    // Map range scan into generic data structure
    range_sensor_t range_sensor;   
    range_scan_t range_scan;
    lidar_t lidar = 
    {
        .range_sensor = &range_sensor,
        .range_scan = &range_scan
    };

    range_sensor.angular_resolution = msg->angle_increment;
    range_sensor.nb_measurements= msg->ranges.size();
    range_sensor.max_angle = msg->angle_max;
    range_sensor.min_angle = msg->angle_min;
    range_sensor.max_distance = msg->range_max;
    range_sensor.min_distance = msg->range_min;

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

    struct timespec tic,toc;
    clock_gettime(CLOCK_MONOTONIC,  &tic);
    // Verify tubes that are available
    barn_motion_tube_.Compute(&range_sensor, &range_scan);
    // ********* Navigation cost function ***********************/
    // Find the waypoint that each horizon has to aim for
    std::vector<point2d_t> vec_goal;
    
    // how you iterate in c++ without using i
    for (auto &mp_params: barn_motion_tube_.params_.vec_mp_params)
    {
        // imagine a projected distance divide by intervals -> gives you the number of waypoints required -> index of waypoint on global path
        int index = (int) ceil(
                (fabs(mp_params.forward_velocity*mp_params.time_horizon)+goal_distance_offset_)/
                GLOBAL_PLANNER_WAYPOINT_SAMPLING_INTERVAL);
        index = std::min(index, waypoints_.number_of_points-1);
        vec_goal.push_back(waypoints_.points[index]);
    }
    // Compute cost for each maneuver
    int best_i_index=-1, best_j_index;
    double best_cost=10;
    double total_cost = 0;
    int number_of_voters = 0;
    int number_at_lowest_level = 0;

    std::vector<double> best_forward_vel_;
    std::vector<double> best_angular_vel_;
    std::vector<double> best_cost_;

    // reverse direction
    for(int i=0; i<barn_motion_tube_.motion_tube_.size(); i++)
    {
        double time_horizon = barn_motion_tube_.params_.vec_mp_params[i].time_horizon;
        size_t nb_tubes_ith_horizon = barn_motion_tube_.motion_tube_[i].size();
        for(size_t j=0; j<nb_tubes_ith_horizon; j++)
        {
            cost_[i][j] = 100;
            if(barn_motion_tube_.availability_[i][j]){
                cost_[i][j] = compute_points_distance(&barn_motion_tube_.final_position_[i][j],
                    &vec_goal[i]);
                number_at_lowest_level++;
            }

            
        }        
        std::vector<double>::iterator best_cost_ith_horizon = std::min_element(cost_[i].begin(), cost_[i].end());
        int index = std::distance(cost_[i].begin(), best_cost_ith_horizon);
        if(*best_cost_ith_horizon < 0.5  )
        {
            if(*best_cost_ith_horizon < best_cost)
                best_cost = *best_cost_ith_horizon;
            best_i_index = i;
            best_j_index = index;
            unicycle_control_t *local_control = (unicycle_control_t*) 
                barn_motion_tube_.motion_primitive_[best_i_index][best_j_index].control; 
            best_forward_vel_.push_back(local_control->forward_velocity);
            best_angular_vel_.push_back(local_control->angular_rate);
            best_cost_.push_back( std::max(0.5-*best_cost_ith_horizon,0.01) );    
            total_cost += best_cost_[number_of_voters];
            // std::cout << "i: " << i << " j: " << index << " v: " << local_control->forward_velocity <<
            //     " w: " << local_control->angular_rate*180/M_PI << " r: " << local_control->forward_velocity/local_control->angular_rate <<
            //     " cost: " << *best_cost_ith_horizon << " retified cost: " << best_cost_[number_of_voters] <<std::endl;
            number_of_voters++;

        }
    }
    // std::cout <<"----------" << std::endl;
    geometry_msgs::Twist twist_message;
    if(number_of_voters > 0)  {
        double total_weight = 0;
        double v_weight_sum = 0;
        double w_weight_sum = 0;
        // std::cout << "[weight] "; 
        for(int i=0; i<number_of_voters; i++)
        {
            double weight = total_cost/best_cost_[i];
            total_weight += weight;
            v_weight_sum += best_forward_vel_[i]*weight;
            w_weight_sum += best_angular_vel_[i]*weight;
            // std::cout << i << ": " << weight << ", ";
        }
        // std::cout << "total: " << total_weight << std::endl;
        double v = v_weight_sum/total_weight ;
        double w = w_weight_sum/total_weight;
        // std::cout << "chosen v: " << v << "chosen w: " << w << std::endl;
        // std::cout <<"---------------" << std::endl;

        // velcoity profile generation: choices for v_desired based on v as a guidance, v is a desired velocity calculated by weights 
        double v_desired;;
        if (v>0.3){
            v_desired = max_forward_vel;
        }else if (v>0.2){
            v_desired = 0.2;
        }else{
            v_desired = 0.05;
        }
        
        if(v_desired > last_forward_vel){
            v_desired = std::min( last_forward_vel + acceleration*dt, v_desired );
        }
        if(fabs(w) < 1e-3)
        {
            twist_message.linear.x = v_desired;
            twist_message.angular.z = w;
        }else
        {
            double r = v/w;    
            double w_clipped = std::max(std::min(v_desired/r, max_angular_rate), -max_angular_rate);
            twist_message.linear.x = r*w_clipped;
            twist_message.angular.z = w_clipped; 
        }
        last_forward_vel=twist_message.linear.x ;
        last_w_vel = twist_message.angular.z;
    }else 
    {   
        // obstacle avoidance behavior portion

        // logic which causes irrational spinning. However, seldom goes 
        // into this loop with new param. Input brake function here

        if (fabs(last_w_vel) > 1e-1 ){
            twist_message.linear.x = -.0;
            twist_message.angular.z = .75*last_w_vel/fabs(last_w_vel);
        }

        if (goal_arrived) BrakeWithAccLimits();
        
        
        // when footprint is increased, this case gets triggered mostly
        else if (!goal_arrived){
            std::cout<< "avoidance" << std::endl; 
            twist_message.linear.x = -0.3;
            twist_message.angular.z = 0.0;
        }
        last_forward_vel=twist_message.linear.x;
    }


    // std::cout << "message! Best cost: " ;
    // std::cout << best_cost << " v: " << twist_message.linear.x << " w: " << twist_message.angular.z << 
    //     " r: " << twist_message.linear.x/twist_message.angular.z << std::endl;
    cmd_vel_pub_.publish(twist_message);

    // Predicted Path forward

    // // uses the most recent time horizon as reference
    double time_horizon_path = barn_motion_tube_.params_.vec_mp_params.back().time_horizon;

    // std::cout<< "horizon period: " << time_horizon_path << std::endl;
    double sim_granularity = 0.02;
    int min_no_pts = int(time_horizon_path/sim_granularity);
    double dt = time_horizon_path/min_no_pts;

    nav_msgs::Path predicted_path;
    predicted_path.header.frame_id = "odom";

    // Initialize variables for projection
    double projected_x = pose_odom_.x;
    double projected_y = pose_odom_.y;
    double projected_yaw = pose_odom_.yaw;

    // Loop through time to calculate future positions
    for (double i = 0; i < time_horizon_path; i += dt) {
        // Calculate projected positions
        projected_x += twist_message.linear.x * dt * cos(projected_yaw);
        projected_y += twist_message.linear.x * dt * sin(projected_yaw);
        projected_yaw += twist_message.angular.z * dt;

        geometry_msgs::PoseStamped poses;

        // Fill pose message
        poses.header.frame_id = "odom";
        poses.header.stamp = ros::Time::now();
        poses.pose.position.x = projected_x;
        poses.pose.position.y = projected_y;
        poses.pose.position.z = 0.0;

        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, projected_yaw);

        poses.pose.orientation.x = q.x();
        poses.pose.orientation.y = q.y();
        poses.pose.orientation.z = q.z();
        poses.pose.orientation.w = q.w();

        // Add pose to the predicted path
        predicted_path.poses.push_back(poses);
    }

    path_pub_.publish(predicted_path);

    // std::cout << "-----------------------------------------------"<< std::endl;
    // clock_gettime(CLOCK_MONOTONIC,  &toc);
    double time_diff = ((double)toc.tv_sec - tic.tv_sec) + (toc.tv_nsec - tic.tv_nsec)*1e-9; 
    // std::cout << "time in ms: " << time_diff*1e3 << " " << 1e3 << std::endl;

    // // barn_motion_tube_.PrintAvailability();
    if(print==false){
        return;
    }
    print=false;
}

double BarnMotionTubeNode::sign(double x){
    return x < 0.0 ? -1.0 : 1.0;
  }

void BarnMotionTubeNode::BrakeWithAccLimits(){
    geometry_msgs::Twist twist_msg;

    // Define the limits
    AccLimits acc_limits = {2.0, 5.0, 5.0, 20.0};

    double vx = sign(vel_odom_.x) * std::max(0.0, fabs(vel_odom_.x - acc_limits.acc_lim_x * acc_limits.sim_period));
    double vy = sign(vel_odom_.y) * std::max(0.0, fabs(vel_odom_.y - acc_limits.acc_lim_y * acc_limits.sim_period));
    double vth = sign(vel_odom_.yaw) * std::max(0.0, (fabs(vel_odom_.yaw) - acc_limits.acc_lim_theta * acc_limits.sim_period));
    twist_msg.linear.x = vx;
    twist_msg.linear.y = vy;
    twist_msg.angular.z = vth;
    cmd_vel_pub_.publish(twist_msg);
}


//! @breif: Subscribe to odometry in Odom frame
void BarnMotionTubeNode::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Odom frame
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double yaw = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
    // Transform to world frame
    //double c = cos(yaw), s= sin(yaw);
    pose_odom_.x = x; 
    pose_odom_.y = y;
    pose_odom_.yaw =  yaw;
    vel_odom_.x = msg->twist.twist.linear.x;
    vel_odom_.y = msg->twist.twist.linear.y;
    vel_odom_.yaw = msg->twist.twist.angular.z;


        // You can use these values to compute the distance or other relative metrics
        // distance_to_goal = sqrt(goal_x_in_base * goal_x_in_base + goal_y_in_base * goal_y_in_base);
        distance_to_goal = sqrt((goal_.pose.position.x - pose_odom_.x) * (goal_.pose.position.x - pose_odom_.x) + (goal_.pose.position.y - pose_odom_.y) * (goal_.pose.position.y - pose_odom_.y));
        std::cout<< distance_to_goal << std::endl;

        // Check if the robot has arrived at the goal. This distance threshold 
        // must be longer than the length of a generated motion tube / before distance_to_goal runs out
        if (distance_to_goal < 0.5) {
            goal_arrived = true;
            std::cout << "goal reaching" <<std::endl;
        } else {
            goal_arrived = false;
            // std::cout << "goal NOT reaching" <<std::endl;

        }

}

void BarnMotionTubeNode::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Store the received goal
    if (msg->pose.position.x != 0.0){
        goal_ = *msg;
        std::cout<< "goal" << goal_ <<std::endl; 
    
}

}

void BarnMotionTubeNode::GlobalNavPlannerCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(new_plan_request==false)
        return;
    double c = cos(pose_odom_.yaw), s = sin(pose_odom_.yaw);
    // @ TODO: Make this more complete, i.e., consider the full pose transform
    // from the lidar to the base_link, not only the y-offset
    point2d_t translation = {
        .x = -(c*pose_odom_.x + s*pose_odom_.y) + LIDAR_OFFSET_X,
        .y = -(-s*pose_odom_.x + c*pose_odom_.y)
    };

    waypoints_.number_of_points = 0;  
    for (auto &pose: msg->poses)
    {
        if(waypoints_.number_of_points < waypoints_.max_number_of_points)
        {
            // Transform points to lidar frame
            waypoints_.points[waypoints_.number_of_points].x = c*pose.pose.position.x + s*pose.pose.position.y + translation.x;
            waypoints_.points[waypoints_.number_of_points].y = -s*pose.pose.position.x + c*pose.pose.position.y + translation.y;
            waypoints_.number_of_points += 1;
        }else
        {
            break;
        }
    }    

}

double wrap_to_pi(double angle)
{
    double angle_wrapped = angle;
    if(angle > M_PI)
    {
        angle_wrapped -= 2*M_PI;
    }
    else if (angle < -M_PI)
    {
        angle_wrapped += 2*M_PI;
    }
    return angle_wrapped;
}

pose2d_t TransformPose(pose2d_t &in1, pose2d_t &in2)
{
    double c = cos(in1.yaw), s = sin(in2.yaw);
    return {c*in2.x - s*in2.y + in1.x, 
        s*in2.x + c*in2.y + in1.y, 
        wrap_to_pi(in1.yaw + in2.yaw)};
}

double compute_points_distance(point2d_t *p1, point2d_t *p2)
{
    return sqrt( pow(p1->x-p2->x, 2) + pow(p1->y-p2->y, 2) );
}


void PrintPoint2dArray(point2d_array_t *points)
{
    for(size_t i=0; i<points->number_of_points; i++)
    {
        std::cout << points->points[i].x << " " << points->points[i].y << " ";
    }   
    std::cout << std::endl;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "barn_motion_tube");
    BarnMotionTubeNode nav_node;
    
    ros::spin();


    // BarnMotionTube barn_motion_tube;
    // range_sensor_t range_sensor; 
    // range_sensor.angular_resolution = 0.0065540750511;
    // range_sensor.nb_measurements = 0;
    // range_sensor.max_angle = 2.3561899662;
    // range_sensor.min_angle = -2.3561899662;
    // range_sensor.max_distance = 30.0;
    // range_sensor.min_distance = 0.10;

    // barn_motion_tube.Configure(&range_sensor);

    // for(size_t i=0; i<barn_motion_tube.motion_tube_.size(); i++)
    // {
    //     for(size_t j=0; j<barn_motion_tube.motion_tube_[i].size(); j++)
    //     {
    //         print_cartesian_points(&barn_motion_tube.motion_tube_[i][j].cartesian);
    //         print_beams(&barn_motion_tube.motion_tube_[i][j].sensor_space);
    //     }

    // }



    return 0;
}

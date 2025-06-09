# Mobile Robots Local planners / Motion planners


## Installation
- Ensure you are in your <workspace>/src directory
```
git clone https://github.com/Evintjh/Mobile-Robots-local-planners.git
cd ..
catkin_make
source devel/setup.bash
```

## Running the dwa planner
```
roslaunch jackal_navigation odom_navigation_demo.launch
```

### Static obstacle avoidance
https://github.com/user-attachments/assets/6a519872-aaf7-4537-be57-62861156e937

### Dynamic obstacle avoidance


https://github.com/user-attachments/assets/aac910f6-c672-43e8-96e2-c9476e6feaa4



## Testing freespace motion tube / Simplified MPC
```
roslaunch free_space_motion_tube_ros move_base_free_space.launch
```

### Static obstacle avoidance
https://github.com/user-attachments/assets/edda02c9-661a-4803-a44e-269968921c8c

### Dynamic obstacle avoidance


https://github.com/user-attachments/assets/7f986112-e313-4ea0-952c-a40370b5e3fa




## Results
- Static obstacle avoidance
  - freespace motion tube is less conservative than DWA
  - freespace motion tube has lower chances of getting stucked at local extrema comapared to DWA
 
- Dynamic obstacle avoidance
  - freespace motion tube is less reactive & more predictive than DWA planner, allowing smoother navigation

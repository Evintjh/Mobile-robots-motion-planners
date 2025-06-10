# Mobile Robots Motion Planners for Cluttered Environments
![6132134901575697787](https://github.com/user-attachments/assets/4671f018-010d-4d8a-a2d6-75458fa75a22)


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
roslaunch jackal_navigation dwa_odom.launch
```

## Running the teb planner
```
roslaunch jackal_navigation teb_odom.launch
```

## Running the simplified MPC planner
```
roslaunch freespace_motion_tube_ros move_base_free_space.launch
```


## Results
### Narrow corridor & Static obstacle avoidance
![6080210709767114380](https://github.com/user-attachments/assets/eac9b394-ef3a-45a8-a8f3-edc5f035250b)


#### Simplified MPC


https://github.com/user-attachments/assets/4463730d-b8c5-4bd4-812e-46becfe3c1da



### TEB


https://github.com/user-attachments/assets/f42308ac-a9b9-48d2-8ccd-24cccb0bd3fc






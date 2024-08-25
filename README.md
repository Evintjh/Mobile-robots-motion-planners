# Isaac-Sim-local-planners-testing

## Requirements
- Ensure that you have isaac sim properly installed

## Installation
- Ensure you are in your <workspace>/src directory
```
git clone https://github.com/Evintjh/Isaac-Sim-local-planners-testing.git
cd ..
catkin_make
source devel/setup.bash
```

## Run simulator
```
/isaac-sim/python.sh /<your directory>/jackal_launcher.py
```

## Testing dwa planner
```
roslaunch jackal_navigation odom_navigation_demo.launch
```


https://github.com/user-attachments/assets/6a519872-aaf7-4537-be57-62861156e937


## Testing freespace motion tube / Simplified MPC
```
roslaunch free_space_motion_tube_ros move_base_free_space.launch
```


https://github.com/user-attachments/assets/edda02c9-661a-4803-a44e-269968921c8c

## Results
- Static obstacle avoidance
  - freespace motion tube is less conservative than DWA
  - freespace motion tube has lower chances of getting stucked at local extrema comapared to DWA

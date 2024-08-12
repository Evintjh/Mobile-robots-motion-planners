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

## Testing freespace motion tube / Simplified MPC
```
roslaunch free_space_motion_tube_ros move_base_free_space.launch
```

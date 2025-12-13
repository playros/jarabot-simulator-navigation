# Jarabot Simulator (ROS2 Humble)

This repository is the jarabot simulator workspace that 
allows testing of the robot's movements in RViz2 without 
the physical jarabot hardware.

## Package Structure

- src/jarabot_sim
- src/jarabot_sim_interfaces

## Build Procedure

```bash
cd jarabot_sim_ws
colcon build
source install/setup.bash
ros2 launch jarabot_sim jarabot_simulator.launch.py  

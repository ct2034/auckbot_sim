The Auckbot Simulation
===========

This packages contain the Simulation of the AuckBot. In particular:

- __`auckbot_control`__: Obsolete package which will be removed soon
- __`auckbot_description`__: The description of the robot for simulation
- __`auckbot_gazebo`__: Simulation settings and environment
- __`auckbot_navigation`__: Navigation launchfiles and maps (mainly wrappers of available packages)
- __`auckbot_teleop`__: Little tool to manually control the robot

## Tutorial

To use this software, follow these steps:<br/>
(It is recommended to have look at some [tutorials](http://wiki.ros.org/ROS/Tutorials) if you are new to ROS)

1. Create a ROS workspace
  - `mkdir -p ~/ros/auckbot_ws/src`
  - `cd ~/ros/auckbot_ws/src`
  - `catkin_init_workspace`
  
2. Download this source
  - `git clone git@github.com:ct2034/auckbot_sim.git`
  
3. Build the code
  - `cd ~/ros/auckbot_ws`
  - `catkin_make`


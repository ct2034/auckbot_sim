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

4. (OPTIONAL) If you want to use rtabmap:
  - Get the code and compile it according to the available [tutorial](https://code.google.com/p/rtabmap/wiki/Installation#ROS_version)
  - Notes:
    - In case you experience an error like <br/>
    > No rule to make target /usr/lib/x86_64-linux-gnu/libGL.so"<br/>
    create a link to the correct location of the library `sudo ln -s /usr/lib/libGL.so /usr/lib/x86_64-linux-gnu/libGL.so`

cmake_minimum_required(VERSION 2.8.3)
project(auckbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  message_generation
)

add_message_files(
  FILES
  MotorCurrents.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auckbot_teleop
   CATKIN_DEPENDS roscpp message_runtime
   DEPENDS eigen
)

add_executable(current_pub src/current_pub.cpp)

target_link_libraries(
  current_pub
  ${catkin_LIBRARIES}
)

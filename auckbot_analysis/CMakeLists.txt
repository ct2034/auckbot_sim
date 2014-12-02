cmake_minimum_required(VERSION 2.8.3)
project(auckbot_analysis)

#set(MongoClient_DIR "$HOME/mongo-client-install")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/auckbot_sim/auckbot_analysis/")

MESSAGE( ">> CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )

find_package(catkin REQUIRED COMPONENTS
  auckbot_gazebo
  geometry_msgs
  move_base_msgs
  roscpp
  tf
)
find_package(Boost COMPONENTS system filesystem thread)
find_package(OpenSSL)
find_package(MongoClient REQUIRED)

catkin_package(
   CATKIN_DEPENDS auckbot_gazebo geometry_msgs move_base_msgs roscpp tf
   DEPENDS Boost MongoClient
)

MESSAGE( ">> MongoClient_INCLUDE_DIR: " ${MongoClient_INCLUDE_DIR} )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${MongoClient_INCLUDE_DIR}
)

add_executable(nav_analysis src/nav_analysis.cpp)

MESSAGE( ">> MongoClient_LIBRARIES: " ${MongoClient_LIBRARIES} )

target_link_libraries(nav_analysis
  ${catkin_LIBRARIES}
  ${MongoClient_LIBRARIES} 
  ${Boost_LIBRARIES}
)
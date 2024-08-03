cmake_minimum_required(VERSION 3.0.2)
project(map_changer)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    geometry_msgs
    waypoint_manager_msgs
    nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_changer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(map_changer_node src/map_changer_node.cpp)

target_link_libraries(map_changer_node yaml-cpp ${catkin_LIBRARIES})
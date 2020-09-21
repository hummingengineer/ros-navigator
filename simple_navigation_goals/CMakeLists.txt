cmake_minimum_required(VERSION 3.0.2)
project(simple_navigation_goals)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES simple_navigation_goals
 CATKIN_DEPENDS actionlib move_base_msgs roscpp
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})


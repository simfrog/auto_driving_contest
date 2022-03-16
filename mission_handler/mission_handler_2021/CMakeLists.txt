cmake_minimum_required(VERSION 3.0.2)
project(mission_handler_2021)

add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  autoware_msgs
  traffic_light_msgs
  move_base_msgs
  actionlib
  traffic_area_msgs
  tf
  pcl_ros
  pcl_conversions
)
catkin_package(

)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(mission_manager
  include/mission_handler_2021/Mission.h
  src/IsPointInPolygon.cpp
  include/mission_handler_2021/IsPointInPolygon.h
  src/MissionManager.cpp
)
add_dependencies(mission_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_manager
  ${catkin_LIBRARIES}
)

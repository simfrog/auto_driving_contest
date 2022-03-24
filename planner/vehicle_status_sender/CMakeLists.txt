cmake_minimum_required(VERSION 3.0.2)
project(vehicle_status_sender)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  erp42_msgs
  roscpp
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_status_sender
  src/vehicle_status_sender.cpp
)
add_dependencies(vehicle_status_sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_status_sender
  ${catkin_LIBRARIES}
)
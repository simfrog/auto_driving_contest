cmake_minimum_required(VERSION 3.0.2)
project(odom_imu_merger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)
add_executable(odom_imu_merger
  src/odom_imu_merger.cpp
)
add_dependencies(odom_imu_merger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_imu_merger
  ${catkin_LIBRARIES}
)
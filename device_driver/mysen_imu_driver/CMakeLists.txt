cmake_minimum_required(VERSION 3.0.2)
project(mysen_imu_driver)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp serial)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(driver_node
  src/driver_node.cpp
  src/driver_core.cpp
)
add_dependencies(driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver_node
  ${catkin_LIBRARIES}
)
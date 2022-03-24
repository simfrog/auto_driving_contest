cmake_minimum_required(VERSION 3.0.2)
project(yolo_traffic_light)

find_package(catkin REQUIRED COMPONENTS 
message_generation 
roscpp 
sensor_msgs 
cv_bridge 
std_msgs 
image_transport 
autoware_msgs 
traffic_light_msgs 
traffic_area_msgs
)

find_package(autoware_build_flags REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
message_generation
roscpp
sensor_msgs
cv_bridge
std_msgs
image_transport
autoware_msgs
traffic_light_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  autoware_msgs
  traffic_light_msgs
)


include_directories(
  ${autoware_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(yolo_traffic_light
  src/yolo_traffic_light.cpp
)
add_dependencies(yolo_traffic_light ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yolo_traffic_light
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
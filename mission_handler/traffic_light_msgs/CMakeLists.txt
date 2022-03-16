cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_msgs)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
add_message_files(FILES
  Light.msg
)
generate_messages(DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
)

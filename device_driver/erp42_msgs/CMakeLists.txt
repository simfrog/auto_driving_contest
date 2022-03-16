cmake_minimum_required(VERSION 3.0.2)
project(erp42_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs
)
add_message_files(FILES
  erp42_to_pc.msg
  pc_to_erp42.msg
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

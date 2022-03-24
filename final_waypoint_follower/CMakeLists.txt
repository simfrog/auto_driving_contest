cmake_minimum_required(VERSION 2.8.3)
project(final_waypoint_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O -Wall -std=c++14 -fpermissive)

find_package(autoware_build_flags REQUIRED)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        sensor_msgs
        image_transport
        roscpp
        tf
        jsk_recognition_utils
        eigen_conversions
        tf_conversions
        jsk_topic_tools
        image_geometry
        autoware_msgs
        visualization_msgs
        # Start
        pcl_ros
        geometry_msgs
        pcl_conversions
        op_ros_helpers
        vector_map_msgs
)

catkin_package(
        CATKIN_DEPENDS
        cv_bridge
        sensor_msgs
        image_transport
        roscpp
        tf
        autoware_msgs
        jsk_recognition_utils
        eigen_conversions
        tf_conversions
        jsk_topic_tools
        image_geometry
        visualization_msgs
        # Start
        pcl_ros
        geometry_msgs
        pcl_conversions
        op_ros_helpers
        vector_map_msgs
        # End 
)


###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

##############################################################################
add_executable(final_waypoint_follower
  src/final_waypoint_follower.cpp
)

#################################################################################
add_dependencies(final_waypoint_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(final_waypoint_follower
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

##############################################################################
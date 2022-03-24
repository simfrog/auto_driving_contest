cmake_minimum_required(VERSION 2.8.3)
project(lidar_quadtree_cluster)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O -Wall -std=c++14 -fpermissive)

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
add_executable(quadtree_segmentation
  nodes/quadtree_segmentation/quadtree_segmentation_node.cpp
  nodes/quadtree_segmentation/quadtree_segmentation.cpp
  nodes/quadtree_segmentation/quadtree.cpp
)
add_dependencies(quadtree_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadtree_segmentation
  ${catkin_LIBRARIES}
)
##############################################################################
add_executable(track_driving
  nodes/track_driving/track_driving.cpp
)
add_dependencies(track_driving ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(track_driving
  ${catkin_LIBRARIES}
)
##############################################################################
add_executable(map2velodyne
  nodes/track_driving/map2velodyne.cpp
)
add_dependencies(map2velodyne ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map2velodyne
  ${catkin_LIBRARIES}
)
##############################################################################

install(TARGETS  quadtree_segmentation map2velodyne
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)



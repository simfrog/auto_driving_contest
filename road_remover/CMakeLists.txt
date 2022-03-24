cmake_minimum_required(VERSION 2.8.3)
project(road_remover)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O -Wall -std=c++14 -fpermissive)

find_package(autoware_build_flags REQUIRED)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        roscpp
        tf
        eigen_conversions
        tf_conversions
        pcl_ros
        geometry_msgs
        pcl_conversions
)

catkin_package(
        CATKIN_DEPENDS
        sensor_msgs
        roscpp
        tf
        eigen_conversions
        tf_conversions
        pcl_ros
        geometry_msgs
        pcl_conversions
)


###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

##############################################################################
add_executable(road_remover
  src/road_remover.cpp
)
add_dependencies(road_remover ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(road_remover
  ${catkin_LIBRARIES}
)

##############################################################################


install(TARGETS road_remover  
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)



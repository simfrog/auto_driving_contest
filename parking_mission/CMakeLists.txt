cmake_minimum_required(VERSION 2.8.3)
project(parking_mission)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O -Wall -std=c++14 -fpermissive)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        sensor_msgs
        roscpp
        tf_conversions
        tf
        geometry_msgs
        pcl_conversions
)

catkin_package(
        CATKIN_DEPENDS
        cv_bridge
        image_transport
        sensor_msgs
        roscpp
        tf_conversions
        tf
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
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)


##############################################################################
add_executable(parking_mission
  src/parking_mission.cpp
)
add_dependencies(parking_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parking_mission
  ${catkin_LIBRARIES}
)

##############################################################################


install(TARGETS parking_mission 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)



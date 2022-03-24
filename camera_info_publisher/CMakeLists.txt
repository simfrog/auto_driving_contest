cmake_minimum_required(VERSION 2.8.3)
project(camera_info_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O -Wall -std=c++14 -fpermissive)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        roscpp
)

catkin_package(
        CATKIN_DEPENDS
        sensor_msgs
        roscpp
)


###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

##############################################################################
add_executable(camera_info_publisher
  src/camera_info_publisher.cpp
)
add_dependencies(camera_info_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_info_publisher
  ${catkin_LIBRARIES}
)

##############################################################################


install(TARGETS  camera_info_publisher 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)



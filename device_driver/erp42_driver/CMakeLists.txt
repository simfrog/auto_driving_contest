cmake_minimum_required(VERSION 3.0.2)
project(erp42_driver)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS roscpp erp42_msgs serial tf std_msgs autoware_msgs)
catkin_package(
	CATKIN_DEPENDS
	roscpp

)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(erp42_main
  src/erp42_main.cpp
)
add_dependencies(erp42_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(erp42_main
  ${catkin_LIBRARIES}
)

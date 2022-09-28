cmake_minimum_required(VERSION 3.0.2)
project(imsogyeong_action)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  gazebo_ros
  cv_bridge
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES imsogyeong_action
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf gazebo_ros
  DEPENDS GAZEBO
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(stanley_drive src/stanley_drive.cpp)
add_dependencies(stanley_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stanley_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS stanley_drive
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

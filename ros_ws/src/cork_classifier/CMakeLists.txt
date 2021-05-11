cmake_minimum_required(VERSION 3.0.2)
project(cork_classifier)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()


## Generate services in the 'srv' folder
add_service_files(
  FILES
  ClassifyCork.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cork_classifier
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#catkin_install_python(PROGRAMS src/classifier_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)




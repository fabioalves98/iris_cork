cmake_minimum_required(VERSION 2.8.3)
project(iris_cork)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  sensor_msgs
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/PCLCork.cfg
)

find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui 
  imgproc 
  imgcodecs
)

find_package(PCL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_executable(iris_cork src/main.cpp src/box.cpp src/pcl_functions.cpp src/cork_iris.cpp)
target_link_libraries(iris_cork ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_scene src/test_scene.cpp)
target_link_libraries(test_scene ${catkin_LIBRARIES})

add_dependencies(iris_cork ${PROJECT_NAME}_gencfg)

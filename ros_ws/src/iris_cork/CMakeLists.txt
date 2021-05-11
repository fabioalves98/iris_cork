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
  geometry_msgs
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

add_message_files(
    FILES
    CorkInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

find_package(PCL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(iris_sami REQUIRED)
find_package(cork_classifier REQUIRED)

include_directories(${iris_sami_INCLUDE_DIRS})
include_directories(${cork_classifier_INCLUDE_DIRS})

add_executable(iris_cork src/main.cpp src/box.cpp src/pcl_functions.cpp src/cork_iris.cpp)
target_link_libraries(iris_cork ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_scene src/test_scene.cpp)
target_link_libraries(test_scene ${catkin_LIBRARIES})

add_dependencies(iris_cork ${PROJECT_NAME}_gencfg)



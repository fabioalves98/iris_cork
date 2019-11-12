cmake_minimum_required(VERSION 2.8.3)
project(cork_iris)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  sensor_msgs
)

## Eventualmente esta versao nao precisa de estar aqui
find_package(OpenCV 3.2.0 REQUIRED COMPONENTS
  core
  highgui 
  imgproc 
  imgcodecs
)

find_package(libfreenect REQUIRED)
find_package(PCL 1.2 REQUIRED)


catkin_package(

)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${FREENECT_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(capture src/capture.cpp src/ImageParser.cpp src/box.cpp src/DepthParser.cpp)
target_link_libraries(capture ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(pcl_points src/pcl_points.cpp)
target_link_libraries(pcl_points ${catkin_LIBRARIES} ${PCL_LIBRARIES})

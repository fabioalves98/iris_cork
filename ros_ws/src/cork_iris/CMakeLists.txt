cmake_minimum_required(VERSION 2.8.3)
project(cork_iris)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Eventualmente esta versao nao precisa de estar aqui
find_package(OpenCV 3.2.0 REQUIRED COMPONENTS
  core
  highgui 
  imgproc 
  imgcodecs
)

find_package(libfreenect REQUIRED)

catkin_package(

)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${FREENECT_INCLUDE_DIRS})

## 
add_executable(capture src/capture.cpp)
target_link_libraries(capture ${OpenCV_LIBS} ${catkin_LIBRARIES} ${FREENECT_LIBRARIES})
add_executable(load src/load.cpp)
target_link_libraries(load ${OpenCV_LIBS} ${catkin_LIBRARIES})


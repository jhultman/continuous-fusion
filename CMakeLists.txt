cmake_minimum_required(VERSION 2.8.3)
project(continuous_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  cv_bridge
  image_transport
  message_runtime
  sensor_msgs
)

find_package(
  OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibrator 
    src/calibpublisher.cpp
    src/kittireader.cpp
    src/calibration.cpp
)

add_dependencies(calibrator
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(calibrator 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)
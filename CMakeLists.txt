cmake_minimum_required(VERSION 2.8.3)
project(continuous_fusion)

find_package(catkin REQUIRED
    cmake_modules
    COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    sensor_msgs
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
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

find_package(
    PCL
)

find_package(
    Eigen REQUIRED
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(kitticalib 
    src/calibpublisher.cpp
    src/kittireader.cpp
    src/calibration.cpp
)

add_dependencies(kitticalib
    ${catkin_EXPORTED_TARGETS} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(kitticalib
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)

add_executable(kitticamera
    src/imagepublisher.cpp
    src/kittireader.cpp
    src/calibration.cpp
)

add_dependencies(kitticamera
    ${catkin_EXPORTED_TARGETS} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(kitticamera
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)

add_executable(kittivelo
    src/velopublisher.cpp
    src/kittireader.cpp
    src/calibration.cpp
)

add_dependencies(kittivelo
    ${catkin_EXPORTED_TARGETS} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(kittivelo
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(continuousfusion
    src/continuousfusion.cpp
    src/kittireader.cpp
    src/calibration.cpp
    src/bevprojector.cpp
)

add_dependencies(continuousfusion
    ${catkin_EXPORTED_TARGETS} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(continuousfusion
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
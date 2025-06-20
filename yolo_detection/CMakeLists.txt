cmake_minimum_required(VERSION 3.0.2)
project(yolo_detection)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

## System dependencies
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## catkin specific configuration
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    cv_bridge 
    message_runtime
  DEPENDS 
    OpenCV
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(bounding_box_listener src/bounding_box_listener.cpp)

## Add cmake target dependencies of the executable
add_dependencies(bounding_box_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(bounding_box_listener
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Mark executables for installation
install(TARGETS bounding_box_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS scripts/yolo_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
cmake_minimum_required(VERSION 3.0.2)
project(dynamixelcontrol)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  sensor_msgs
  yolo_detection
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamixelcontrol
  CATKIN_DEPENDS dynamixel_sdk roscpp sensor_msgs yolo_detection
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(class_test_ax 
src/class_test_ax.cpp
src/DynamixelControl/Motor.cpp
src/DynamixelControl/AXMotor.cpp)
target_link_libraries(class_test_ax ${catkin_LIBRARIES})

add_executable(class_test_mx
src/class_test_mx.cpp
src/DynamixelControl/Motor.cpp
src/DynamixelControl/MXMotor.cpp)
target_link_libraries(class_test_mx ${catkin_LIBRARIES})

add_executable(dxcontrol_test
src/dxcontrol_test.cpp
src/DynamixelControl/Motor.cpp
src/DynamixelControl/AXMotor.cpp
src/DynamixelControl/MXMotor.cpp
src/DynamixelControl/DynamixelControl.cpp)
target_link_libraries(dxcontrol_test ${catkin_LIBRARIES})

add_executable(param
src/param.cpp
src/DynamixelControl/Motor.cpp
src/DynamixelControl/AXMotor.cpp
src/DynamixelControl/MXMotor.cpp
src/DynamixelControl/DynamixelControl.cpp)
target_link_libraries(param ${catkin_LIBRARIES})

add_executable(param2
src/param2.cpp
src/DynamixelControl/Motor.cpp
src/DynamixelControl/AXMotor.cpp
src/DynamixelControl/MXMotor.cpp
src/DynamixelControl/DynamixelControl.cpp)
target_link_libraries(param2 ${catkin_LIBRARIES})
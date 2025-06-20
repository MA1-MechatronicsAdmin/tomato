cmake_minimum_required(VERSION 3.0.2)
project(tomato_kobuki)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kobuki_msgs
  sensor_msgs
  roscpp
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kobuki_operation
  src/main.cpp
  src/kobuki_operation.cpp
)
target_link_libraries(kobuki_operation
  ${catkin_LIBRARIES}
)


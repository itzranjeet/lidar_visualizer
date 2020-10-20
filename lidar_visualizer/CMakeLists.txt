#------------------------------------------------------------------------------
# Copyright (C) 2020, KPIT
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.0.2)
project(lidar_visualizer)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR /usr/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS roscpp rospy
  std_msgs
  message_generation
  cv_bridge
  image_transport
)
add_message_files(FILES message.msg )

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OpenCV_LIBS})

add_executable(Node src/lidar.cpp)
target_link_libraries(Node ${catkin_LIBRARIES})
add_dependencies(Node lidar_visualizer_generate_messages_cpp)




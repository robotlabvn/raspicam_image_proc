cmake_minimum_required(VERSION 2.8.3)
project(raspicam_image_proc)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


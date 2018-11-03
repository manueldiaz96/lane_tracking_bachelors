cmake_minimum_required(VERSION 2.8.3)
project(self_driving_car_thesis)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  camera_info_manager
  sensor_msgs
)


catkin_package()


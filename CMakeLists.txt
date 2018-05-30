cmake_minimum_required(VERSION 2.8.3)
project(self_driving_car_thesis)

add_compile_options(-std=c++11)

set(OpenCV_DIR /usr/local/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  image_transport
  roscpp
  std_msgs
)


catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(edgesNode src/Edges.cpp)
 target_link_libraries(edgesNode
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(linesNode src/Lines2.cpp)
 target_link_libraries(linesNode
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

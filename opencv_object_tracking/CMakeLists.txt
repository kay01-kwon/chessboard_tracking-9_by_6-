cmake_minimum_required(VERSION 2.8.3)
project(opencv_object_tracking)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)


add_message_files(
   FILES
 )


generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   sensor_msgs
 )

 catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv_object_tracking
  CATKIN_DEPENDS message_generation roscpp std_msgs geometry_msgs message_runtime image_transport cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(circle_tracker src/circle_tracker.cpp)
add_executable(chessboard_tracker src/chessboard_tracker.cpp)

target_link_libraries(circle_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(chessboard_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

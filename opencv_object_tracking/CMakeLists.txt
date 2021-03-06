cmake_minimum_required(VERSION 2.8.3)
project(opencv_object_tracking)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
  sensor_msgs
)
find_package(OpenCV REQUIRED)


add_message_files(
   FILES
   position_publish.msg
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
  CATKIN_DEPENDS message_generation roscpp std_msgs geometry_msgs message_runtime image_transport cv_bridge sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(object_filter src/object_filter.cpp)
add_executable(circle_tracker src/circle_tracker.cpp)
add_executable(chessboard_tracker src/chessboard_tracker.cpp)
add_executable(aruco_tracker src/aruco_tracker.cpp)
add_executable(chess_pid_control src/chessboard_pid.cpp include/opencv_object_tracking/chessboard_pid.cpp)
add_executable(yaw_pickup src/yaw_pickup.cpp include/opencv_object_tracking/orientation_pickup.cpp)


target_link_libraries(object_filter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(circle_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(chessboard_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(aruco_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(chess_pid_control ${catkin_LIBRARIES})
target_link_libraries(yaw_pickup ${catkin_LIBRARIES})

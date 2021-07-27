# object_detection_2d
This source code to take object detection from 2D and point out the position in 3D coordinate of chessboard from RealSense D435 camera under ROS platform.<br>
 <br>
Process to use this code: <br>

 $ catkin_make <br>
 $ roslaunch realsense2_camera rs_rgbd.launch Or $ roslaunch realsense2_camera rs_camera.launch<br> 
 $ rosrun opencv_object_tracking chessboard_tracking

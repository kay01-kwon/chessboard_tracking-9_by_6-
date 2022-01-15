# Chessboard and aruco marker tracking

# Pose estimation through checker board marker
<img src="opencv_object_tracking/image/detected_coordinate.png" width="680" height="480" />

First of all, calibrate the D435i camera through opencv.

When you get the camera matrix and distortion coefficients, plug them into the matrix.

Necessary Device: D435i, pxhawk4

Topic Subscribe to /mavros/imu/mag and /camera/color/image_raw

Precision: within 2 mm

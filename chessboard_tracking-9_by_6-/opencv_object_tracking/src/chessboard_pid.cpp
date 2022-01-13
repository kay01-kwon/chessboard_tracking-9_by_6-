#include <opencv_object_tracking/chessboard_pid.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"chessboard_pi");
    pid_control pid_control;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
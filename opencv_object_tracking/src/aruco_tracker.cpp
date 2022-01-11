#include <opencv_object_tracking/aruco_tracker.hpp>

int main(int argc, char** argv){

    ros::init(argc, argv, "aruco_tracker");

    std::cout<<"Aruco Tracker\n";
    
    ArucoTracker aruco_tracker;

    std::cout<<"Declaration\n";

    aruco_tracker.initialization();

    aruco_tracker.subscriberSetting();
    aruco_tracker.publisherSetting();
    std::cout<<"Starting subscribe to image msg\n";

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
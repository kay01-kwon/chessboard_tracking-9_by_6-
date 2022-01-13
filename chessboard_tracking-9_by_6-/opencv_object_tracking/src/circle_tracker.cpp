#include <opencv_object_tracking/circle_tracker.hpp>

int main(int argc, char** argv){

    ros::init(argc, argv, "circle_tracker");

    std::cout<<"Circle Tracker\n";
    
    Circle_Tracker circle_tracker;

    std::cout<<"Declaration\n";

    circle_tracker.subscriberSetting();
    std::cout<<"Starting subscribe to image msg";

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
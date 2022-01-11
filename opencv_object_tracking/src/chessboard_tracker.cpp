#include <opencv_object_tracking/chessboard_tracker.hpp>

int main(int argc, char** argv){

    ros::init(argc, argv, "chessboard_tracker");

    std::cout<<"Chessboard Tracker\n";
    
    ChessboardTracker chessboard_tracker;

    std::cout<<"Declaration\n";

    chessboard_tracker.initialization();

    chessboard_tracker.subscriberSetting();
    chessboard_tracker.publisherSetting();
    std::cout<<"Starting subscribe to image msg\n";

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
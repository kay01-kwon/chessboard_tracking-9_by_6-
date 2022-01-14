#include <opencv_object_tracking/orientation_pickup.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"orientation_pickup");
    OriPick ori_pickup;

    ros::spin();

//    return 0;
}
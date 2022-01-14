#include "orientation_pickup.h"

OriPick::OriPick()
{
    imu_subscriber = nh.subscribe("/mavros/imu/data",1,&OriPick::imu_callback,this);
}

void OriPick::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    qx = imu_msg->orientation.x;
    qy = imu_msg->orientation.y;
    qz = imu_msg->orientation.z;
    qw = imu_msg->orientation.w;

    current_yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
    angleNormalizer(current_yaw);
    printout();

}

void OriPick::angleNormalizer(double & yaw)
{
    yaw = atan2(sin(yaw),cos(yaw));
}

void OriPick::printout()
{
    cout<<"current_yaw : "<<current_yaw<<endl;
}

OriPick::~OriPick()
{
}
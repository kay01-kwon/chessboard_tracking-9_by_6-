#ifndef ORIENTATION_PICKUP_H
#define ORIENTATION_PICKUP_H
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
using std::cout;
using std::endl;

class OriPick{

    public:

    OriPick();

    void imu_callback(const sensor_msgs::MagneticFieldConstPtr &imu_msg);

    void printout();
    void angleNormalizer(double& yaw);
    ~OriPick();

    private:
    ros::NodeHandle nh;
    ros::Subscriber imu_subscriber;

    double qx, qy,qz,qw;
    double current_yaw;

    double mx, my;
};




#endif
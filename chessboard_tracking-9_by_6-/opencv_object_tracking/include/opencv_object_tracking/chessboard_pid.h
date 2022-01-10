#ifndef CHESSBOARD_PID_H
#define CHESSBOARD_PID_H

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Transform.h>
#include <ethercat_test/vel.h>
#include <std_msgs/Int8.h>

#define _USE_MATH_DEFINES

using geometry_msgs::Transform;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

using ethercat_test::vel;

using std::cout;
using std::endl;

class pid_control{

    public:

    // Constructor
    pid_control();

    void callback_pose(const geometry_msgs::TransformConstPtr & pose_msg);
    
    void callback_control_enable(const std_msgs::Int8ConstPtr & control_msg);

    void error_calculation();

    void cmd_vel_calculation();

    Vector4d inverse_kinematics(Vector3d vel_cmd);

    Vector4d clamp(Vector4d motor_input);
    
    void cmd_vel_publish();
    
    Matrix3d getRotMat(double yaw);

    // Destructor
    ~pid_control();

    private:
    ros::NodeHandle nh_;

    ros::Publisher publisher_cmd_vel;
    ros::Subscriber subscriber_pose;

    // ROS Time info
    double curr_time;
    double last_time = ros::Time::now().toSec();
    double dt = 0.01;

    // Variables
    Vector3d goal_pose;

    Vector3d chessboard_pose;
    double yaw;

    Vector3d p_err;
    Vector3d p_err_prev;
    Vector3d dpdt_err;
    Vector3d integral_p_err;

    Vector3d a_cmd;
    Vector3d v_cmd;
    Vector3d v_cmd_prev;

    // Gains
    MatrixXd Kp = MatrixXd(3,3);
    MatrixXd Kd = MatrixXd(3,3);
    MatrixXd Ki = MatrixXd(3,3);

    // Controller PD Gain
    double Kp_xy = 5;
    double Kp_yaw = 1;

    double Kd_xy = 10;
    double Kd_yaw = 0;
    
    double Ki_xy = 0.001;
    double Ki_yaw = 0.001;

    // mobile robot spec
    const double wheel_radious = 0.1520/2.0;
    const double l_a = 0.2170;
    const double l_b = 0.1687;
    const double l_sep = l_a + l_b;

    // Motor spec and unit conversion ratio
    const double gear_ratio = 74.5;
    const int motor_vel_lim = 100;
    const double radps_to_rpm = 60/2.0/M_PI;
    MatrixXd InvJacob = MatrixXd(4,3);
    Vector4d motor_vel_input;

    double qx, qy, qz, qw;

    int8_t control_enable;
    const int8_t inactive = (int8_t) 0;
    const int8_t use_now = (int8_t) 1;
    const int8_t success = (int8_t) 2;
};



#endif
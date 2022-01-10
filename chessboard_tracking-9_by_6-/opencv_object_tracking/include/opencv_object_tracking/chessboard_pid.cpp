#include "chessboard_pid.h"

// Constructor
pid_control::pid_control()
{
    goal_pose << 0.350, 0.050, 0;
    
    p_err.setZero();
    p_err_prev.setZero();
    dpdt_err.setZero();
    integral_p_err.setZero();

    a_cmd.setZero();
    v_cmd.setZero();
    v_cmd_prev.setZero();

    nh_.getParam("Kp_xy",Kp_xy);
    nh_.getParam("Kp_yaw",Kp_yaw);

    nh_.getParam("Kd_xy",Kd_xy);
    nh_.getParam("Kd_yaw",Kd_yaw);

    nh_.getParam("Ki_xy",Ki_xy);
    nh_.getParam("Ki_yaw",Ki_yaw);

    cout<<"Gain Parameters setup"<<endl;
    
    cout<<"Proportional Gain"<<endl;

    cout<<"Kp xy: "<<Kp_xy<<endl;
    cout<<"Kp yaw: "<<Kp_yaw<<endl;

    cout<<"Derivative Gain"<<endl;

    cout<<"Kd xy: "<<Kd_xy<<endl;
    cout<<"Kd yaw: "<<Kd_yaw<<endl;

    cout<<"Integral Gain"<<endl;

    cout<<"Ki xy: "<<Ki_xy<<endl;
    cout<<"Ki yaw: "<<Ki_yaw<<endl;

    Kp<<Kp_xy, 0, 0,
        0, Kp_xy, 0,
        0, 0, Kp_yaw;

    Kd<<Kd_xy, 0, 0,
        0, Kd_xy, 0,
        0, 0, Kd_yaw;

    Ki<<Ki_xy, 0, 0,
        0, Ki_xy, 0,
        0, 0, Ki_yaw;

    InvJacob<<1.0/wheel_radious, -1.0/wheel_radious, -l_sep/wheel_radious,
            1.0/wheel_radious, 1.0/wheel_radious, l_sep/wheel_radious,
            1.0/wheel_radious, -1.0/wheel_radious, l_sep/wheel_radious,
            1.0/wheel_radious, 1.0/wheel_radious, -l_sep/wheel_radious;

    motor_vel_input.setZero();

    cout<<"Publisher setup"<<endl;
    
    publisher_cmd_vel = nh_.advertise<vel>("/input_msgs",1);

    cout<<"Subscriber setup"<<endl;

}

void pid_control::callback_pose(const geometry_msgs::TransformConstPtr& pose_msg)
{

    qw = pose_msg->rotation.w;
    qx = pose_msg->rotation.x;
    qy = pose_msg->rotation.y;
    qz = pose_msg->rotation.z;

    yaw = asin(2*(qw*qy-qz*qx));

    chessboard_pose << (pose_msg->translation.x),
                        (pose_msg->translation.y),
                        -yaw;
}

void pid_control::callback_control_enable(const std_msgs::Int8ConstPtr& control_msg)
{
    control_enable = control_msg->data;

}

void pid_control::error_calculation()
{
    p_err = goal_pose - chessboard_pose;
    dpdt_err = (p_err - p_err_prev)/dt;
    integral_p_err = (p_err + integral_p_err)*dt;
    p_err_prev = p_err;
}

void pid_control::cmd_vel_calculation()
{
    a_cmd = Kp*p_err;
    a_cmd = - a_cmd;
    //v_cmd = v_cmd_prev + a_cmd*dt;
    //v_cmd_prev = v_cmd;
    v_cmd = Kp*p_err;
    v_cmd = -v_cmd;
    v_cmd = getRotMat(yaw)*v_cmd;
}

Vector4d pid_control::inverse_kinematics(Vector3d vel_cmd)
{
    return InvJacob * vel_cmd;
}

Vector4d pid_control::clamp(Vector4d motor_input)
{
    double max_val = motor_input.maxCoeff();
    double max_val2 = fabs(motor_input.minCoeff());

    if( max_val2 > max_val)
        max_val = max_val2;
    
    if( max_val > motor_vel_lim)
        motor_input = motor_input/max_val*motor_vel_lim;

    return motor_input;   
}

void pid_control::cmd_vel_publish()
{
    if(control_enable != inactive || control_enable != success){
        vel motor_vel_pub;

        curr_time = ros::Time::now().toSec();
        dt = curr_time - last_time;

        error_calculation();
        cmd_vel_calculation();

        motor_vel_input = inverse_kinematics(v_cmd);
        motor_vel_input = clamp(motor_vel_input*gear_ratio*radps_to_rpm);

        std::cout<<p_err<<std::endl;
        std::cout<<std::endl;

        if(fabs(p_err(0))<0.002 && fabs(p_err(1))<0.002 && fabs(p_err(2))<0.0017)
        {
            control_enable = success;
            motor_vel_input.setZero();
            v_cmd.setZero();
            v_cmd_prev.setZero();
            std::cout<<"Stop"<<std::endl;
        }

        motor_vel_pub.velocity[0] = (int) motor_vel_input(0);
        motor_vel_pub.velocity[1] = (int)-motor_vel_input(1);
        motor_vel_pub.velocity[2] = (int)-motor_vel_input(2);
        motor_vel_pub.velocity[3] = (int) motor_vel_input(3);
    
        publisher_cmd_vel.publish(motor_vel_pub);

        last_time = curr_time;
    }
}

Matrix3d pid_control::getRotMat(double yaw)
{
    Matrix3d R;
    R << cos(yaw), sin(yaw), 0,
        -sin(yaw), cos(yaw), 0,
        0, 0, 1;
    
    return R;
}
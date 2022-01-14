#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using sensor_msgs::ImageConstPtr;
using sensor_msgs::PointCloud2;
using std::vector;
using geometry_msgs::Transform;
using Eigen::Vector3d;
using Eigen::Matrix3d;

using tf::TransformListener;
using tf::StampedTransform;

static const std::string windowName1 = "Gray Image";

class ChessboardTracker{
    
    public:
    void initialization();
    void imageCallback(const ImageConstPtr& img_msg);
    void calcPose(double position[3]);
    void subscriberSetting();
    void publisherSetting();
    Matrix3d getRot(double yaw);
    void angleNormalizer(double & yaw);

    private:
    ros::NodeHandle nh;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Publisher publisher_pose;

    image_transport::Subscriber image_subscriber;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
        
    cv::Size boardSize;
    cv::Mat camera_matrix;
    cv::Mat distortionCoeffs;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat tf;
    cv::Mat translation;
    cv::Mat rotation;
    cv::Mat R;
    std::vector<cv::Point3f> corners_info;

    double theta;
    double s[3];
    double trace;
    float squareSize;
    Vector3d position_;
    Matrix3d R_;

    double qx, qy, qz, qw;
    double yaw;

};

void ChessboardTracker::initialization()
{
    squareSize = 0.022;
    boardSize.width = 9;
    boardSize.height = 6;

    camera_matrix = (cv::Mat1d(3,3) << 448.06375, 0, 276.77853, 
                                        0, 446.82352, 222.37746, 
                                        0, 0, 1);
    
    distortionCoeffs = (cv::Mat1d(1,5) << 0.0587196, 
                                            -0.0542081, 
                                            -0.000444068, 
                                            -0.00850087, 
                                            0.098648);

    tf = (cv::Mat1d(3,3) << 0, 0, 1, 
                            -1, 0, 0,
                            0, -1, 0);

    std::cout<<"camera matrix info\n";
    std::cout<<camera_matrix<<std::endl;

    std::cout<<"distortion coefficients\n";
    std::cout<<distortionCoeffs<<std::endl;



    for(int i = 0; i < boardSize.height; i++)
        for(int j = 0; j < boardSize.width; j++)
            corners_info.push_back(cv::Point3f(float(j*squareSize),
                                                float(i*squareSize),
                                                0));

    //std::cout<<"Corner info(Global coordinate):\n";
    //std::cout<<corners_info<<std::endl;

}

void ChessboardTracker::imageCallback(const ImageConstPtr& img_msg)
{

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv bridge exception: %s",e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image,img,cv::IMREAD_COLOR);
    std::vector<cv::Point2f> detected_corners;

    cv::imshow("original image",img);

    bool found = cv::findChessboardCorners(cv_ptr->image, boardSize,detected_corners);

//    cv::imshow("image",img);


    if(found == false){
        std::cout<<"Not detected\n";
        return;
    }

    

    //cv::drawChessboardCorners(img, boardSize,detected_corners,found);
    cv::solvePnP(corners_info,
    detected_corners,
    camera_matrix,
    distortionCoeffs,
    rvec,tvec);

    translation = tf*tvec;

    double* position = (double*) translation.data;

    //std::cout<<"\n";


    std::vector<cv::Point3f> obj_pts;
    obj_pts.push_back(cv::Point3f(0,0,0));
    obj_pts.push_back(cv::Point3f(0.066,0,0));
    obj_pts.push_back(cv::Point3f(0,0.066,0));
    obj_pts.push_back(cv::Point3f(0,0,0.066));



    cv::Rodrigues(rvec,R);
    rotation = R;
    calcPose(position);

    std::vector<cv::Point2f> imagePoints;

    cv::projectPoints(obj_pts,rvec,tvec,
    camera_matrix,distortionCoeffs,imagePoints);

    cv::line(img,imagePoints.at(0),imagePoints.at(1),cv::Scalar(0,0,255),5,8,0);
    cv::line(img,imagePoints.at(0),imagePoints.at(2),cv::Scalar(0,255,0),5,8,0);
    cv::line(img,imagePoints.at(0),imagePoints.at(3),cv::Scalar(255,0,0),5,8,0);

    cv::imshow("detected coordinate",img);

    cv::waitKey(1);
}

void ChessboardTracker::calcPose(double position[3])
{
    Transform tf;


    try
    {
        listener.lookupTransform("map","base_footprint",ros::Time(0),transform);
        tf::Quaternion quat = transform.getRotation();
        yaw = tf::getYaw(quat);
        angleNormalizer(yaw);
        qx = quat.x();
        qy = quat.y();
        qz = quat.z();
        qw = quat.w();

    }
    catch(const tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }


    double* Rotation = (double*) rotation.data;


    position_ << position[0], position[1], position[2];
/**
    trace = Rotation[0] + Rotation[4] + Rotation[8];
    theta = acos((trace-1.0)/2.0);

    double magnitude;

    s[0] = (Rotation[7] - Rotation[5])/sin(theta)/2.0;
    s[1] = (Rotation[2] - Rotation[6])/sin(theta)/2.0;
    s[2] = (Rotation[3] - Rotation[1])/sin(theta)/2.0;
    
    double pitch;

    qw = cos(theta/2.0);
    qx = sin(theta/2.0)*s[0];
    qy = sin(theta/2.0)*s[1];
    qz = sin(theta/2.0)*s[2];

    if(theta == 0)      
    {
        qw = 1.0;
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
    }

    tf.rotation.w = qw;
    tf.rotation.x = qx;
    tf.rotation.y = qy;
    tf.rotation.z = qz;
    
    
    double roll, yaw;

    pitch = asin(2*(qw*qy-qz*qx));
    roll = atan2(2*(qw*qx+qy*qz),1.0-(qx*qx+qy*qy));
    yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
**/
//    position_ = getRot(yaw)*position_;

    tf.translation.x = position_(0);
    tf.translation.y = position_(1);
    tf.translation.z = position_(2);
    
    std::cout<<"Position (m): "<<"  "<<position_(0)<<"  "<<position_(1)<<"  "<<position_(2)<<std::endl;
    
    //std::cout<<"rotation matrix"<<std::endl;
    //std::cout<<rotation<<std::endl;
    //std::cout<<trace<<std::endl;

    //std::cout<<theta<<std::endl;

    //std::cout<<"s: "<<s[0]<<"\n"<<s[1]<<"\n"<<s[2]<<std::endl;
    //std::cout<<sqrt(s[0]*s[0]+s[1]*s[1]+s[2]*s[2])<<std::endl;
    //std::cout<<"roll: "<<yaw*180.0/3.14159<<"  ";
    //std::cout<<"pitch: "<<roll*180.0/3.141592<<"  ";
    
    //std::cout<<"yaw: "<<pitch<<std::endl;



    publisher_pose.publish(tf);
}

void ChessboardTracker::subscriberSetting()
{
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("/camera/color/image_raw",1,&ChessboardTracker::imageCallback,this);
}

void ChessboardTracker::publisherSetting()
{
    publisher_pose = nh.advertise<Transform>("chess_board_pose",1);
}

Matrix3d ChessboardTracker::getRot(double yaw)
{
    Matrix3d rot;
    rot<<1, 0, 0,
        0, cos(yaw), -sin(yaw),
        0, sin(yaw), cos(yaw);

    return rot;
}

void ChessboardTracker::angleNormalizer(double &yaw)
{
    yaw = atan2(sin(yaw),cos(yaw));
}
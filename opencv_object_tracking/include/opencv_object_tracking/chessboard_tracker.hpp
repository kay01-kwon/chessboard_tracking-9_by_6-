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

using sensor_msgs::ImageConstPtr;
using sensor_msgs::PointCloud2;
using std::vector;
using geometry_msgs::Transform;
static const std::string windowName1 = "Gray Image";

class ChessboardTracker{
    
    public:
    void initialization();
    void imageCallback(const ImageConstPtr& img_msg);
    void calcPose(double position[3]);
    void subscriberSetting();
    void publisherSetting();


    private:
    ros::NodeHandle nh;
    ros::Publisher publisher_pose;
    image_transport::Subscriber image_subscriber;

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
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv bridge exception: %s",e.what());
        return;
    }

    cv::Mat img;

    cv::cvtColor(cv_ptr->image,img,cv::IMREAD_GRAYSCALE);
    std::vector<cv::Point2f> detected_corners;

    bool found = cv::findChessboardCorners(cv_ptr->image, boardSize,detected_corners);

    cv::imshow("image",cv_ptr->image);


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
    //rotation = tf*R;
    rotation = R;
    //std::cout<<R<<std::endl;
    //std::cout<<"\n";
    calcPose(position);

    std::vector<cv::Point2f> imagePoints;

    cv::projectPoints(obj_pts,rvec,tvec,
    camera_matrix,distortionCoeffs,imagePoints);

    cv::line(cv_ptr->image,imagePoints.at(0),imagePoints.at(1),cv::Scalar(0,0,255),5,8,0);
    cv::line(cv_ptr->image,imagePoints.at(0),imagePoints.at(2),cv::Scalar(0,255,0),5,8,0);
    cv::line(cv_ptr->image,imagePoints.at(0),imagePoints.at(3),cv::Scalar(255,0,0),5,8,0);

    cv::imshow("detected coordinate",cv_ptr->image);

    cv::waitKey(1);
}

void ChessboardTracker::calcPose(double position[3])
{
    Transform tf;
    /**
    std::cout<<"translation vector\n";
    std::cout<<position[0]<<std::endl;
    std::cout<<position[1]<<std::endl;
    std::cout<<position[2]<<std::endl;
    **/
   //std::cout<<"rotation matrix"<<std::endl;
   //std::cout<<rotation<<std::endl;

   double* Rotation = (double*) rotation.data;

    trace = Rotation[0] + Rotation[4] +Rotation[8];
    theta = acos((trace-1)/2.0);

    double magnitude;

   s[0] = (Rotation[7] - Rotation[5])/sin(theta)/2.0;
   s[1] = (Rotation[2] - Rotation[6])/sin(theta)/2.0;
   s[2] = (Rotation[3] - Rotation[1])/sin(theta)/2.0;

    magnitude = sqrt(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]);

    s[0] = s[0]/magnitude;
    s[1] = s[1]/magnitude;
    s[2] = s[2]/magnitude;
    

   double qx, qy, qz, qw;
   double psi;

   qw = cos(theta/2.0);
   qx = sin(theta/2.0)*cos(s[0]);
   qy = sin(theta/2.0)*cos(s[1]);
   qz = sin(theta/2.0)*cos(s[2]);

   if(theta ==0)      
    {
        qw = 1.0;
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
    }

    tf.translation.x = position[0];
    tf.translation.y = position[1];
    tf.translation.z = position[2];

    tf.rotation.w = qw;
    tf.rotation.x = qx;
    tf.rotation.y = qy;
    tf.rotation.z = qz;

    psi = atan2(2.0*(qw*qz+qx*qy),(1-2.0*(qy*qy+qz*qz)));

    //std::cout<<"rotation matrix"<<std::endl;
    //std::cout<<rotation<<std::endl;
    //std::cout<<"s: "<<sqrt(s[0]*s[0]+s[1]*s[1]+s[2]*s[2])<<std::endl;
    //std::cout<<"s: "<<s[0]<<"\n"<<s[1]<<"\n"<<s[2]<<std::endl;

    std::cout<<"yaw: "<<psi*180.0/3.141592<<std::endl;

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
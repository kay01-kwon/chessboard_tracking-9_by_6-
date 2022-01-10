#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <vector>

#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::PointCloud2;
using std::vector;
static const std::string windowName1 = "Gray Image";

class Circle_Tracker{

    public:
    void imageCallback(const ImageConstPtr& img_msg);
    void subscriberSetting();

    private:
    ros::NodeHandle nh;
    ros::Publisher publisher_pose;
    image_transport::Subscriber image_subscriber;
    //ros::Subscriber image_subscriber;

};

void Circle_Tracker::imageCallback(const ImageConstPtr& img_msg)
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

    Mat hsv_img;
    Mat hsv_img_filtered;
    Mat gray;
    cvtColor(cv_ptr->image,hsv_img,COLOR_BGR2HSV);
    inRange(hsv_img,Scalar(20,20,20),Scalar(100,200,80),hsv_img_filtered);
    GaussianBlur(hsv_img_filtered,gray,Size(3,3),2,2);




//    cvtColor(cv_ptr->image,gray,COLOR_BGR2GRAY);
//    medianBlur(gray,gray,3);

    vector<Vec3f> circles;

    HoughCircles(gray,circles,HOUGH_GRADIENT,1,
    gray.rows,100,30,0,0);

    for(size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0],c[1]);
        int radius = c[2];
        circle(gray,center,1,Scalar(0,100,100),3,LINE_AA);
        circle(gray,center,radius,Scalar(255,0,255),3,LINE_AA);
        
        std::cout<<"x :"<<c[0]<<"  y : "<<c[1];
        std::cout<<"\n";
    }

    imshow(windowName1,cv_ptr->image);

    //imshow("Test HSV Filtered",hsv_img_filtered);
    //imshow("HSV",hsv_img);
    waitKey(3);

    

}

void Circle_Tracker::subscriberSetting()
{
    
    image_transport::ImageTransport it(nh);

    image_subscriber = it.subscribe("/camera/color/image_raw",1,&Circle_Tracker::imageCallback,this);
    
   //image_subscriber = nh.subscribe("/camera/color/image_raw",1,&Circle_Tracker::imageCallback);
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>  
#include <sstream>
using namespace cv;
using namespace std;

Mat colorImg;
Mat depthImg;
int flag = 1 ;
 
Mat src; 
Mat src_gray; 
Mat canny_output;
int thresh = 30; 
int max_thresh = 255; 
int getpix(Mat src);

void imageCallbackrgb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    colorImg = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert image1 from '%s' to 'bgr8'. ", msg->encoding.c_str());
  }
}
void imageCallbackdepth(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    depthImg = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert image1 from '%s' to 'bgr8'. ", msg->encoding.c_str());
  }  
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_reciver");
  ros::NodeHandle nh;
  //ros::NodeHandle nh2;
  cv::namedWindow("rgb",CV_WINDOW_NORMAL);
  cv::namedWindow("depth",CV_WINDOW_NORMAL);
  cv::namedWindow( "gray", CV_WINDOW_NORMAL);
  cv::namedWindow( "canny", CV_WINDOW_NORMAL );
  cv::namedWindow( "Contours", CV_WINDOW_NORMAL);
  cv::startWindowThread();
  image_transport::ImageTransport it1(nh);//使用ImageTransport方法创建图像发布者和订阅者，就像我们使用NodeHandle方法创建一般ROS发布者和订阅者一样
  //image_transport::ImageTransport it2(nh2);
  image_transport::Subscriber sub1 = it1.subscribe("/camera/rgb/image_raw", 1, imageCallbackrgb);
  image_transport::Subscriber sub2 = it1.subscribe("/camera/depth/image_raw", 1, imageCallbackdepth);
//  imwrite("/home/lee/catkin_ws/src/showimg/build/output.txt",depthImg);
  while (ros::ok())
  {
    ros::spinOnce();
     
    if(colorImg.data){
	  cvtColor( colorImg, src_gray, CV_BGR2GRAY );
	  Canny( src_gray, canny_output, thresh, thresh*3, 3 ); 	
      imshow("rgb",colorImg);
	  imshow( "gray", src_gray );
	  imshow( "canny", canny_output );  
    }
    if(depthImg.data){
      imshow("depth",depthImg);
    }
	  waitKey(10); 	
  }
  return 0;
  cv::destroyWindow("rgb");
  cv::destroyWindow("depth");
  cv::destroyWindow("gray");
  cv::destroyWindow("canny");
  cv::destroyWindow("Contours");
}
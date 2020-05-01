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
//定义Canny边缘检测图像 
Mat canny_output;
vector<vector<Point> > contours; 	
vector<Vec4i> hierarchy; 	
//Mat contours;
Mat drawing;
int thresh = 30; 
int max_thresh = 255; 

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

int getpix()
{
 if(colorImg.data){
	  cvtColor( colorImg, src_gray, CV_BGR2GRAY );//灰度化 	
    GaussianBlur( src, src, Size(3,3), 0.1, 0, BORDER_DEFAULT ); 	
	  blur( src_gray, src_gray, Size(3,3) ); //滤波 	
	
	  Canny( src_gray, canny_output, thresh, thresh*3, 3 ); 		//利用canny算法检测边缘 

	findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); 	//查找轮廓 	
	
	vector<Moments> mu(contours.size() ); 	//计算轮廓矩 	
	for( int i = 0; i < contours.size(); i++ ) 	
	{ 
		mu[i] = moments( contours[i], false ); 
	} 	
 
	vector<Point2f> mc( contours.size() ); 	//计算轮廓的质心 
	for( int i = 0; i < contours.size(); i++ ) 	
	{ 
		mc[i] = Point2d( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
	}  	
    
	drawing = Mat::zeros( canny_output.size(), CV_8UC3 ); 		//画轮廓及其质心并显示 	
	for( int i = 0; i< contours.size(); i++ ) 	
	{ 		
		Scalar color = Scalar( 255, 0, 0); 		
		drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() ); 		
		circle( drawing, mc[i], 5, Scalar( 0, 0, 255), -1, 8, 0 );		 		
		rectangle(drawing, boundingRect(contours.at(i)), cvScalar(0,255,0)); 			
		char tam[100]; 
		sprintf(tam, "(%0.0f,%0.0f)",mc[i].x,mc[i].y); 
		putText(drawing, tam, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255,0,255),1); 	
  }	
    imshow("rgb",colorImg);
	  imshow( "gray", src_gray );
	  imshow( "canny", canny_output );  
    imshow( "Contours", drawing ); 	
    }
    if(depthImg.data){
      imshow("depth",depthImg);
    }
	  waitKey(10); 	
    return 0;
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
  while (ros::ok())
  {
    ros::spinOnce();
    getpix ();
  
  cv::destroyWindow("rgb");
  cv::destroyWindow("depth");
  cv::destroyWindow("gray");
  cv::destroyWindow("canny");
  cv::destroyWindow("Contours");
}
}
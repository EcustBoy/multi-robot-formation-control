#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
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
int thresh = 30; 
int max_thresh = 255; 
int getpix(string imgstr);

void imageCallbackrgb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    colorImg = cv_ptr->image;
   // imshow("view1",colorImg);
    
    if (flag)
    {
      imwrite("/home/qing/my_prj/ROS_MBOT_SIM_1/src/color.png",colorImg);
      const string imgpath = "/home/qing/my_prj/ROS_MBOT_SIM_1/src/color.png";
      getpix ( imgpath );
      //flag = 0;           
    }    
    imshow("view1",colorImg);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
void imageCallbackdepth(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    depthImg = cv_ptr->image;
    
    imwrite("/home/qing/my_prj/ROS_MBOT_SIM_1/src/depth.png",depthImg);
    imshow("view2",depthImg);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
int getpix(string imgstr)
{ 	    
//	src = imread( "/home/lee/projects/midext/color.png" ,CV_LOAD_IMAGE_COLOR ); 	//注意路径得换成自己的
  
  src = imread( imgstr ,CV_LOAD_IMAGE_COLOR ); 
	cvtColor( src, src_gray, CV_BGR2GRAY );//灰度化 	
	GaussianBlur( src, src, Size(3,3), 0.1, 0, BORDER_DEFAULT ); 	
	blur( src_gray, src_gray, Size(3,3) ); //滤波 	
	namedWindow( "image", CV_WINDOW_AUTOSIZE ); 	
 
	imshow( "image", src ); 	
	moveWindow("image",20,20); 	
	//定义Canny边缘检测图像 	
	Mat canny_output; 	
	vector<vector<Point> > contours; 	
	vector<Vec4i> hierarchy; 	
	//利用canny算法检测边缘 	
 
	Canny( src_gray, canny_output, thresh, thresh*3, 3 ); 	
	namedWindow( "canny", CV_WINDOW_AUTOSIZE ); 	
	imshow( "canny", canny_output ); 	
	moveWindow("canny",550,20); 	
 
	//查找轮廓 	
	findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); 	
	//计算轮廓矩 	
	vector<Moments> mu(contours.size() ); 	
    
	for( int i = 0; i < contours.size(); i++ ) 	
	{ 
		mu[i] = moments( contours[i], false ); 
	} 	
 
	//计算轮廓的质心 	
	vector<Point2f> mc( contours.size() ); 	
	for( int i = 0; i < contours.size(); i++ ) 	
	{ 
		mc[i] = Point2d( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
	}  	
    
	//画轮廓及其质心并显示 	
	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 ); 		
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
 
	namedWindow( "Contours", CV_WINDOW_AUTOSIZE ); 	
	imshow( "Contours", drawing ); 	
	moveWindow("Contours",1100,20); 	
  cout << "mc = :" <<mc << endl;
	waitKey(0); 	
	src.release(); 	
	src_gray.release(); 	
	return 0; 
}
 
 
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_reciver");
  ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  cv::namedWindow("view1",CV_WINDOW_NORMAL);
  cv::namedWindow("view2",CV_WINDOW_NORMAL);
  cv::startWindowThread();
  image_transport::ImageTransport it1(nh1);//使用ImageTransport方法创建图像发布者和订阅者，就像我们使用NodeHandle方法创建一般ROS发布者和订阅者一样
  image_transport::ImageTransport it2(nh2);
  image_transport::Subscriber sub1 = it1.subscribe("/camera/rgb/image_raw", 1, imageCallbackrgb);
  image_transport::Subscriber sub2 = it2.subscribe("/camera/depth/image_raw", 1, imageCallbackdepth);
//  imwrite("/home/lee/catkin_ws/src/showimg/build/output.txt",depthImg);
  
 
  ros::spin();
  cv::destroyWindow("view");
}
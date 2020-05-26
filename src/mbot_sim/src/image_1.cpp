#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <fstream>  
#include <sstream>
using namespace cv;
using namespace std;
#define PI 3.1415926;

Mat colorImg;
Mat depthImg;
int flag = 1 ; 
Mat src; 
Mat src_gray; 
Mat canny_output;
vector<vector<Point> > contours; 	
vector<Vec4i> hierarchy; 	
Mat drawing;
int thresh = 30; 
int max_thresh = 255; 
int getpix(Mat src);
//find the largest contour in fov image of mbot
int max_area = 0;
int max_contour_index = 0;

vector<Moments> mu(1);
vector<Point2f> mc(1); 	//计算轮廓的质心
vector<Point2i> fp(2);  // feature point
Rect bbox;
//camera param
double image_width=1280;
double image_hfov=2.094;
double f=(image_width/2)/tan(image_hfov/2);
int camera_cx=640;
int camera_cy=360;

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
    GaussianBlur( src_gray, src_gray, Size(3,3), 0.1, 0, BORDER_DEFAULT ); 	
	  blur( src_gray, src_gray, Size(3,3) ); //滤波 	
	
	  Canny( src_gray, canny_output, thresh, thresh*3, 3 ); 		//利用canny算法检测边缘 

	  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); 	//查找轮廓 	
    drawing = Mat::zeros( canny_output.size(), CV_8UC3 ); 		//画轮廓及其质心并显示
    //printf("contours size:%d \n",contours.size());
	  //find the largest contour
    if(contours.size()<2){
    //printf("no contours detect!");
      imshow("rgb",colorImg);
	  //imshow( "gray", src_gray );
	  //imshow( "canny", canny_output );  
      imshow( "Contours", drawing ); 
      waitKey(10); 	
    //return 0;
    }
    else
    {
      max_area=0;
      max_contour_index=0;
      for(int i=0;i<contours.size();i++){
      double s=contourArea(contours[i],false);
      if(s>max_area){
        max_area=s;
        max_contour_index=i;
      }
    }
  /*vector<Moments> mu(1);
    vector<Point2f> mc(1); 	//计算轮廓的质心
    vector<Point2f> fp(2);  // feature point*/
    mu[0] = moments(contours[max_contour_index],false);
    mc[0] = Point2d( mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00 );
	 
		Scalar color = Scalar( 255, 0, 0); 		
		drawContours( drawing, contours,  max_contour_index, color, 2, 8, hierarchy, 0, Point() ); 		
		//circle( drawing, mc[0], 5, Scalar( 0, 0, 255), -1, 8, 0 );

    Rect bbox;
    bbox = boundingRect(contours.at( max_contour_index));		 		
	  rectangle(drawing, bbox, cvScalar(0,255,0));
    fp[0]=bbox.tl();
    fp[1]=bbox.br();

    circle( drawing, fp[0],2,Scalar( 0, 0, 255), -1, 8, 0);
    circle( drawing, fp[1],2,Scalar( 0, 0, 255), -1, 8, 0); 			

	  //char tam[100]; 
    char contour_num[1];
    char FP_1[10];
    char FP_2[10];
  
  
		//sprintf(tam, "(%0.0f,%0.0f)",mc[0].x,mc[0].y); 
    //putText(drawing, tam, Point(mc[0].x, mc[0].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255,0,255),1); 	
    
    sprintf(FP_1,"(%d,%d)",fp[0].x,fp[0].y);
    sprintf(FP_2,"(%d,%d)",fp[1].x,fp[1].y);
    putText(drawing, FP_1,fp[0],FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255,0,255),1);
    putText(drawing, FP_2,fp[1],FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255,0,255),1);
    sprintf(contour_num,"%d",contours.size());
    putText(drawing, contour_num,Point(50,50),FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255,0,255),1);
    

    imshow("rgb",colorImg);
	//imshow( "gray", src_gray );
	//imshow( "canny", canny_output );  
    imshow( "Contours", drawing ); 	
    waitKey(10);
    }
    
    }
}

double get_cor2camera(int m,int n)//像素坐标转换为世界坐标
{
//深度图中像素点(m,n)处的深度值
//double point_depth=depthImg.at<float>(x,y);
ROS_DEBUG("compute cor!");
float point_depth=depthImg.ptr<float>(m)[n];

//ushort d = depth.ptr<ushort>(m)[n];
//点P的世界坐标
//p.z = double(point_depth)/camera_factor;
//p.x = (n-camera_cx)*p.z/camera_fx;
//p.y = (m-camera_cy)*p.z/camera_fy;
double z=double(point_depth);
double x=(double(n)-camera_cx)*z/f;
double y=(double(m)-camera_cy)*z/f;

return x,y,z;

}


int main(int argc, char **argv)
{
  double x1,y1,z1,x2,y2,z2;
  ros::init(argc, argv, "image_reciver");
  ros::NodeHandle nh;
  //ros::NodeHandle nh2;
  cv::namedWindow("rgb",CV_WINDOW_NORMAL);
//  cv::namedWindow("depth",CV_WINDOW_NORMAL);
//  cv::namedWindow( "gray", CV_WINDOW_NORMAL);
//  cv::namedWindow( "canny", CV_WINDOW_NORMAL );
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
    getpix();
    
    if(depthImg.data){
    x1,y1,z1=get_cor2camera((fp[0].y+fp[1].y)/2,(fp[0].x+fp[1].x)/2);
  //  v,w=control(x1,y1,z1);
    ROS_DEBUG("%f,%f,%f",x1,y1,z1);
   // x2,y2,z2=get_cor2camera(fp[1].x,fp[1].y);
   // ROS_DEBUG("coordinate in camera frame:%f %f %f %f %f %f",x1,y1,z1,x2,y2,z2);
    }
  }
  return 0;
  cv::destroyWindow("rgb");
//  cv::destroyWindow("depth");
//  cv::destroyWindow("gray");
//  cv::destroyWindow("canny");
  cv::destroyWindow("Contours");
}
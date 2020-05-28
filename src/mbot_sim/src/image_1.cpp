#include <ros/ros.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <fstream>  
#include <sstream>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
//#include <custom_msg/custom_msg.h>
#include <controller.h>
using namespace cv;
using namespace std;
#define PI 3.1415926;
//double w_2,v_2;
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
//model state
double v1,v2,w1,w2;//linear and angular velocity of mbot_1 and mbot_2
double theta1,theta2;//pose angle of mbot_1 and mbot_2
//cor in follower's camera frame
double p_x,p_y,p_z;
//set params
double d=0.2;
double k1=0.01,k2=0.01;
double l_12_d=1;
double phi_12_d=2*3.1415/3;

double controller(double l_12,double theta_1,double theta_2,double phi_12,double v_1,double w_1,double l_12_d,double phi_12_d,double d,double k1,double k2);

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
p_z=double(point_depth);
p_x=(double(n)-camera_cx)*p_z/f;
p_y=(double(m)-camera_cy)*p_z/f;

//return p_x,p_y,p_z;

}

void gazebo_state_callback(const gazebo_msgs::ModelStates& msg){
  //msg.pose();
  //string model_name=msg.name[1];
  geometry_msgs::Pose pose_1=msg.pose[1];// pose of agent_1
  geometry_msgs::Point point_1=pose_1.position;
  geometry_msgs::Quaternion quat_1=pose_1.orientation;
  theta1=2*acos(quat_1.w);//yaw angle
  geometry_msgs::Twist twist_1=msg.twist[1];// twist of agent_1
  geometry_msgs::Vector3 linear_1=twist_1.linear;
  geometry_msgs::Vector3 angular_1=twist_1.angular;
  v1=linear_1.x;
  w1=angular_1.z;

  geometry_msgs::Pose pose_2=msg.pose[2];// pose of agent_2
  geometry_msgs::Point point_2=pose_2.position;
  geometry_msgs::Quaternion quat_2=pose_2.orientation;
  theta2=2*acos(quat_2.w);//yaw angle
  geometry_msgs::Twist twist_2=msg.twist[2];// twist of agent_2
  geometry_msgs::Vector3 linear_2=twist_2.linear;
  geometry_msgs::Vector3 angular_2=twist_2.angular;
  v2=linear_2.x;
  w2=angular_2.z;

}


int main(int argc, char **argv)
{
  double x=0,y=0,z=0;// cor in follower camera frame
  double l_12,phi_12,temp_theta;
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
  geometry_msgs::Twist twist_output;
  geometry_msgs::Twist twist_zero;
  geometry_msgs::Vector3 linear_output;
  geometry_msgs::Vector3 angular_output;

  image_transport::Subscriber sub1 = it1.subscribe("/camera/rgb/image_raw", 1, imageCallbackrgb);
  image_transport::Subscriber sub2 = it1.subscribe("/camera/depth/image_raw", 1, imageCallbackdepth);
  ros::Subscriber sub3 = nh.subscribe("/gazebo/model_states",100,gazebo_state_callback);
  ros::Publisher cmdpub= nh.advertise<geometry_msgs::Twist>("/mbot_1/cmd_vel", 1, true);
//  imwrite("/home/lee/catkin_ws/src/showimg/build/output.txt",depthImg);
  while (ros::ok())
  {
    ros::spinOnce();
    getpix();
    
    if(depthImg.data){
    get_cor2camera((fp[0].y+fp[1].y)/2,(fp[0].x+fp[1].x)/2);
    ROS_DEBUG("%f,%f,%f",p_x,p_y,p_z);
    }
    //send sensor value to controller node
    l_12=sqrt(pow(p_x,2)+pow(p_z,2))-0.2;
    temp_theta=atan(p_x/p_z);
    phi_12=theta2-temp_theta+PI-theta1;
   
    w1,v1=controller(l_12,theta1,theta2,phi_12,v2,w2,l_12_d,phi_12_d,d,k1,k2);

    //pub vel command
    linear_output.x=v1;
    linear_output.y=0;
    linear_output.z=0;
    angular_output.x=0;
    angular_output.y=0;
    angular_output.z=w1;

    twist_output.linear=linear_output;
    twist_output.angular=angular_output;
    cmdpub.publish(twist_output);
    sleep(1);
    linear_output.x=0;
    linear_output.y=0;
    linear_output.z=0;
    angular_output.x=0;
    angular_output.y=0;
    angular_output.z=0;
    twist_zero.linear=linear_output;
    twist_zero.angular=angular_output;
    cmdpub.publish(twist_zero);

  }
  return 0;
  cv::destroyWindow("rgb");
//  cv::destroyWindow("depth");
//  cv::destroyWindow("gray");
//  cv::destroyWindow("canny");
  cv::destroyWindow("Contours");
}
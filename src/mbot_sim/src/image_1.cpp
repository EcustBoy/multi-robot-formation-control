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
double PI=3.1415926;
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
double v_1,v_2,w_1,w_2;//linear and angular velocity of mbot_1 and mbot_2
double theta_1,theta_2;//angle of mbot_1 and mbot_2
double x_1,y_1,x_2,y_2;//pos of mbot_1 and mbot_2
double v1,w1;//control action
//cor in follower's camera frame
double p_x,p_y,p_z;
//set params
double d=0.2;
double k1=0.002,k2=0.001;
double l_12_d=5;
double phi_12_d=4*PI/3;

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
  theta_1=2*acos(quat_1.w);//yaw angle
  x_1=point_1.x;
  y_1=point_1.y;
  geometry_msgs::Twist twist_1=msg.twist[1];// twist of agent_1
  geometry_msgs::Vector3 linear_1=twist_1.linear;
  geometry_msgs::Vector3 angular_1=twist_1.angular;
  v_1=linear_1.x;
  w_1=angular_1.z;

  geometry_msgs::Pose pose_2=msg.pose[2];// pose of agent_2
  geometry_msgs::Point point_2=pose_2.position;
  geometry_msgs::Quaternion quat_2=pose_2.orientation;
  theta_2=2*acos(quat_2.w);//yaw angle
  x_2=point_2.x;
  y_2=point_2.y;
  geometry_msgs::Twist twist_2=msg.twist[2];// twist of agent_2
  geometry_msgs::Vector3 linear_2=twist_2.linear;
  geometry_msgs::Vector3 angular_2=twist_2.angular;
  v_2=linear_2.x;
  w_2=angular_2.z;

}


int main(int argc, char **argv)
{
  double x=0,y=0,z=0;// cor in follower camera frame
  double l_12,phi_12,temp_theta;
  double c_x,c_y;//camera point pos of follower in world frame
  //vector<double> vec1,vec2,vec3;
  double vec1[2],vec2[2],vec3[2];
  double norm1,norm2,norm3;
  double angular_vel_list[127]={-0.2899054139,	-0.1208461567	,-0.0455448147,	-0.0160499987,	-0.0051308012	,-0.0011697481	,0.0002590366,	0.0007747282	,0.0009618542	,0.0010306775	,0.0010568216	,0.0010675138	,0.001072571,	0.001075538	,0.0010776967	,0.0010795115	,0.0010811503	,0.0010826742,	0.0010841051,	0.0010854508	,0.001086714,	0.0010878955,	0.0010889955,	0.001090014	,0.0010909508,	0.0010918058	,0.0010925788	,0.0010932699,	0.0010938789,	0.0010944058,	0.0010948506,	0.0010952133,	0.001095494,	0.0010956927,	0.0010958095,	0.0010958445,	0.001095798,	0.00109567,	0.0010954608,	0.0010951707,	0.0010947998,	0.0010943485,	0.001093817,	0.0010932058,	0.0010925152,	0.0010917455,	0.0010908972,	0.0010899707,	0.0010889665,	0.001087885,	0.0010867268,	0.0010854924,	0.0010841824,	0.0010827972,	0.0010813376,	0.0010798041,	0.0010781973,	0.001076518,	0.0010747668,	0.0010729443,	0.0010710514,	0.0010690886,	0.0010670569,	0.0010649569,	0.0010627894,	0.0010605553,	0.0010582552,	0.0010558901,	0.0010534608,	0.0010509682,	0.001048413,	0.0010457962,	0.0010431187,	0.0010403814,	0.0010375851,	0.0010347308,	0.0010318194,	0.0010288519,	0.0010258291,	0.0010227521,	0.0010196218,	0.0010164391,	0.001013205,	0.0010099206,	0.0010065867,	0.0010032043,	0.0009997745,	0.0009962982,	0.0009927763,	0.00098921,	0.0009856002,	0.0009819478,	0.0009782538,	0.0009745193,	0.0009707452,	0.0009669326,	0.0009630823,	0.0009591954,	0.0009552729,	0.0009513157,	0.0009473248,	0.0009433012,	0.0009392458,	0.0009351597,	0.0009310436,	0.0009268987,	0.0009227258,	0.0009185258,	0.0009142998	,0.0009100485	,0.000905773	,0.0009014742	,0.0008971528	,0.0008928099	,0.0008884463,	0.0008840629,	0.0008796605,	0.00087524,	0.0008708023,	0.0008663482,	0.0008618784,	0.0008573939,	0.0008528955,	0.0008483838,	0.0008438598,	0.0008393242,	0
};
  double linear_vel_list[127]={0.0380403869,	0.0412464669,	0.0420702005,	0.0422848605,	0.0423381019,	0.0423432574	,0.0423328284,	0.0423172261,	0.0422999879,	0.0422823482,	0.0422647428,	0.0422473273,	0.042230158	,0.0422132559	,0.0421966297	,0.0421802832,	0.042164219,	0.0421484387,	0.0421329437,	0.0421177355,	0.0421028151,	0.0420881837,	0.0420738424,	0.0420597922,	0.042046034	,0.0420325686	,0.0420193968	,0.0420065194,	0.041993937,	0.0419816502,	0.0419696595,	0.0419579653,	0.0419465682,	0.0419354683,	0.0419246659,	0.0419141613,	0.0419039547,	0.041894046,	0.0418844354,	0.0418751227,	0.0418661079,	0.0418573908,	0.0418489713	,0.041840849,	0.0418330237	,0.0418254949	,0.0418182622	,0.0418113252	,0.0418046833,	0.0417983359,	0.0417922824,	0.0417865221,	0.0417810542,	0.0417758779,	0.0417709925,	0.041766397	,0.0417620905	,0.041758072,	0.0417543405,	0.041750895,	0.0417477343,	0.0417448573,	0.0417422628,	0.0417399495,	0.0417379162,	0.0417361616,	0.0417346844,	0.0417334831,	0.0417325564,	0.0417319028,	0.0417315207,	0.0417314088,	0.0417315654,	0.0417319889,	0.0417326778,	0.0417336304,	0.0417348451,	0.0417363201,	0.0417380537,	0.0417400442,	0.0417422898,	0.0417447887,	0.0417475391,	0.0417505391,	0.0417537869,	0.0417572807,	0.0417610184,	0.0417649982,	0.0417692181	,0.0417736763	,0.0417783706	,0.0417832992	,0.04178846	,0.041793851,	0.0417994702,	0.0418053156,	0.041811385,	0.0418176765,	0.041824188	,0.0418309174	,0.0418378625	,0.0418450214,	0.0418523919,	0.0418599719,	0.0418677592	,0.0418757519,	0.0418839477,	0.0418923445,	0.0419009402,	0.0419097326,	0.0419187197,	0.0419278993,	0.0419372693,	0.0419468275,	0.0419565718,	0.0419665001	,0.0419766102,	0.0419869001,	0.0419973675,	0.0420080105,	0.0420188268,	0.0420298143,	0.042040971,	0.0420522947,	0.0420637834,	0.0420754349,	0.0504346778
};
  
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
  ros::Publisher cmdpub1= nh.advertise<geometry_msgs::Twist>("/mbot_1/cmd_vel", 1, true);
  ros::Publisher cmdpub2= nh.advertise<geometry_msgs::Twist>("/mbot_2/cmd_vel", 2, true);
//  imwrite("/home/lee/catkin_ws/src/showimg/build/output.txt",depthImg);
  int t=0;
  while (ros::ok())
  {
    t=t+1;
    ros::spinOnce();
    getpix();
    
    if(depthImg.data){
    get_cor2camera((fp[0].y+fp[1].y)/2,(fp[0].x+fp[1].x)/2);
    ROS_DEBUG("%f,%f,%f",p_x,p_y,p_z);
    }
    //send sensor value to controller node
   // l_12=sqrt(pow(p_x,2)+pow(p_z,2))-0.2;
   // temp_theta=atan(p_x/p_z);
   // phi_12=theta2-temp_theta+PI-theta1;
    
    c_x=x_1+d*cos(theta_1);
    c_y=y_1+d*sin(theta_1);
    l_12=sqrt(pow((c_x-x_2),2)+pow((c_y-y_2),2));
    norm1=l_12;
    vec1[0]=(c_x-x_2)/norm1;
    vec1[1]=(c_y-y_2)/norm1;
    norm2=sqrt(pow(d*cos(theta_1),2)+pow(d*sin(theta_1),2));
    vec2[0]=(d*cos(theta_1))/norm2;
    vec2[1]=(d*sin(theta_1))/norm2;
    norm3=sqrt(pow(d*cos(theta_1+PI/2),2)+pow(d*sin(theta_1+PI/2),2));
    vec3[0]=d*cos(theta_1+PI/2)/norm3;
    vec3[1]=d*sin(theta_1+PI/2)/norm3;
    //phi_12=atan(c_y/c_x)-theta_1;
    phi_12=acos(vec1[0]*vec2[0]+vec1[1]*vec2[1]);

    if (acos(vec1[0]*vec3[0]+vec1[1]*vec3[1])<=(PI/2)){
      phi_12=phi_12;
    }
    else
    {
      phi_12=2*PI-phi_12;
    }
    
    //controller(l_12,theta_1,theta_2,phi_12,v_2,w_2,l_12_d,phi_12_d,d,k1,k2);
    if (t<=3){
    linear_output.x=0.6;
    linear_output.y=0;
    linear_output.z=0;
    angular_output.x=0;
    angular_output.y=0;
    angular_output.z=0.4;

    }
    else if(t>3 && t<=6){
    linear_output.x=0.6;
    linear_output.y=0;
    linear_output.z=0;
    angular_output.x=0;
    angular_output.y=0;
    angular_output.z=-0.4;
    }
    else if(t>6){
    linear_output.x=0.6;
    linear_output.y=0;
    linear_output.z=0;
    angular_output.x=0;
    angular_output.y=0;
    angular_output.z=0;
    }
    //pub vel command to mbot_1
    //angular_output.z=3.57*angular_vel_list[t];
   
    twist_output.linear=linear_output;
    twist_output.angular=angular_output;
    cmdpub1.publish(twist_output);
    //pub vel command to mbot_2
    linear_output.x=0.4;
    linear_output.y=0;
    linear_output.z=0;
    angular_output.x=0;
    angular_output.y=0;
    angular_output.z=0;

    twist_output.linear=linear_output;
    twist_output.angular=angular_output;
    cmdpub2.publish(twist_output);
    sleep(1);
  /*  sleep(1);
    linear_output.x=0;
    linear_output.y=0;
    linear_output.z=0;
    angular_output.x=0;
    angular_output.y=0;
    angular_output.z=0;
    twist_zero.linear=linear_output;
    twist_zero.angular=angular_output;
    cmdpub1.publish(twist_zero);*/

  }
  return 0;
  cv::destroyWindow("rgb");
//  cv::destroyWindow("depth");
//  cv::destroyWindow("gray");
//  cv::destroyWindow("canny");
  cv::destroyWindow("Contours");
}
#include <ros/ros.h>
#include <cmath>
#include <fstream>  
#include <sstream>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <unistd.h>
#include <image_1.h>

int controller(x1,y1,z1);
double x1,y1,z1,x2,y2,z2;
double v,w=0;

void controllerCallback_i(const sensor_msgs::ImageConstPtr& msg)
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

void controllerCallback_j(const sensor_msgs::ImageConstPtr& msg)
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


int controller(x1,y1,z1)
{
float r,a,b,c,v,w,e_x,e_y,F_x,F_y;
int m,n=1;//暂定k1,k2的值
int v_max=10;//暂定
r=sqrt(pow(x1,2)+pow(y1,2)+pow(z1,2));//求解距离
a=atan2(x,z)；//求解角度
node_handle::Node_Handle it1(nh);//使用NodeHandle方法创建一般ROS发布者和订阅者
node_handle::Subscriber sub1 = it1.subscribe("controller_i", 1, controllerCallback_i);
node_handle::Subscriber sub2 = it1.subscribe("controller_j", 1, controllerCallback_j);//订阅小车i,j的实时速度方向相对于世界坐标y的角度
b=controller_i-controller_j;//b=beta_i=小车i,j的速度方向相对于世界坐标的方向之差
e_x=r*sin(b-a)-v*t*sin(b);//偏差量=i在领导者框架中的位置-i在t时刻相对于j的期望位移
e_y=r*cos(b-a)-v*t*cos(b);//这里的期望位移我理解为上一时刻的速度*两刻时间差，理论上这个时间差应该极小所以角度不变，这里的时间差应该等于两次运行相差的时间，不知道理解的对不对
F_x=3*pow(e_x,2)-1;
F_y=3*pow(e_y,2)-1;//偶极矢量场
c=atan2(F_y,F_x);

v=-m*Sgn(e_x*cos(b)+e_y*sin(b))*sqrt(pow(e_x,2)+pow(e_y,2))-Sgn(e_x)*v_max；//求取线速度，m,v_max自己选定，m>max{sqrt(sqrt(pow(e_x,2)+pow(e_y,2)),(2*v_max*(1-sqrt(pow(e_x,2)+pow(e_y,2)))+sqrt(pow(e_x,2)+pow(e_y,2)))*pow(r*cos(a),2)*w_max)/(pow(e_x,2)+pow(e_y,2))}
w=-n*(b-c)+c的微分//求取角速度，n自己选定且n>0.5，微分不知道咋写

return v,w;
}





int main(int argc, char **argv)
{
  while (ros::ok())
  {
    ros::spinOnce();
    controller(x1,y1,z1);
    ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("mbot_2/cmd_vel", 1, true);
    twist.linear.x=v;
    twist.angular.z=w;
    cmdpub.publish(twist);
  return 0;
  }
}

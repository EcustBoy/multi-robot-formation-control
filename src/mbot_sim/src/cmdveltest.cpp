#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include<unistd.h>

using namespace std;
int main(int argc,char** argv)
{
    ros::init(argc, argv, "cmdveltest");
     ros::NodeHandle cmdh;
    ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);;
    ros::Rate r(60);
    while(1){
        geometry_msgs::Twist twist;
        geometry_msgs::Vector3 linear;
        linear.x=0.1;
        linear.y=0;
        linear.z=0;
        geometry_msgs::Vector3 angular;
        angular.x=0;
        angular.y=0;
        //直行
        //angular.z=0;
        //转圈
        angular.z=-0.5;
        twist.linear=linear;
        twist.angular=angular;

        cmdpub.publish(twist);
        cout<<"hello"<<endl;
       // ros::spinOnce();
        //r.sleep();
        sleep(1);
    }
    return 0;
}

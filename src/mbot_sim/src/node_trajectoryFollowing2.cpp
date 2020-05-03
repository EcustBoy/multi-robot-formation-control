#include "node_trajectoryFollowing2.h"
#include <math.h>
#define PI 3.141592		//!<Mathematical constant (default value: 3.141592).
#define ANGLE_TO_VELOCITY 1 //!<Angle velocity command = angle * (this constant).
#define APPROX_ITERATIONS 10	//!<Number of iterations in aproximation of intersection.

//Constructor and destructor
NodeTrajectoryFollowing2::NodeTrajectoryFollowing2(ros::Publisher pub, std::queue<MyPoint*> followedTrajectory, double tol, double sp, double targetDist)
{
	trajectory = followedTrajectory;
	if (targetDist < 2*tol){
		ROS_INFO("target distance must be bigger than 2*tolerance");
		exit(1);
	}
	targetDistance = targetDist;
	tolerance = tol;
	speed = sp;
	pubMessage = pub;
	lastRemoved = trajectory.front();
	target = new MyPoint(0.0, 0.0);
	trajectory.pop();//  .pop() 从堆中提取数据
}

NodeTrajectoryFollowing2::~NodeTrajectoryFollowing2()
{
}

//Publisher
void NodeTrajectoryFollowing2::publishMessage(double angleCommand, double speedCommand)
{
	//preparing message
	geometry_msgs::Twist msg;
	
	msg.linear.x = speedCommand;
	msg.angular.z = angleCommand*ANGLE_TO_VELOCITY;
		
	//sending information about message to console
//	ROS_INFO("Sending msg: linear.x=%f, angular.z=%f",msg.linear.x,msg.angular.z);
	
	//publishing message
	pubMessage.publish(msg);
}

//Subscriber
void NodeTrajectoryFollowing2::messageCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	double angleCommand = 0;
	double speedCommand = 0;
	MyPoint* actual = new MyPoint(msg->pose.pose.position.x, msg->pose.pose.position.y);
	printf("robot:x.%f,y%f     ",msg->pose.pose.position.x,msg->pose.pose.position.y);
	findTarget(actual);
	if (closeEnough(actual) == true && trajectory.empty())
	{
		ROS_INFO("GOAL ACHIEVED");
		publishMessage(0.0,0.0);
		exit(0);
	}
	speedCommand = calculateSpeed(actual);   //0.15
	angleCommand = calculateAngle(actual, 2.0*asin(msg->pose.pose.orientation.z), speed);  //计算角速度
	
	//Invoking method for publishing message
	publishMessage(angleCommand, speedCommand);
}

double NodeTrajectoryFollowing2::calculateAngle(MyPoint* actual, double angle, double speed)
{
	double angleCalc;
	double distance;
	double radius;
	double routeDistance;
	double time;
	double angleVelocity;
	angleCalc = actual->getAngle(target)-angle;
	
	//normalizing angle to <-pi; pi>
	if (fabs(angleCalc)>PI)
	{
		angleCalc = angleCalc - copysign(2*PI,angleCalc);
	}
	distance = actual->getDistance(target);
	radius = fabs((distance/2)/(cos(PI/2 - angleCalc)));
	routeDistance = 2*PI*radius*(fabs(angleCalc)/(PI));
	time=routeDistance/speed;
	angleVelocity = 2*angleCalc/time;
	return angleVelocity;
}

bool NodeTrajectoryFollowing2::closeEnough(MyPoint* actual)
{
	double distance;
	distance = actual->getDistance(target);
	if (distance > tolerance)
	{
		return false;
	}
	return true;
}

double NodeTrajectoryFollowing2::calculateSpeed(MyPoint* actual)
{
	return speed;
}

void NodeTrajectoryFollowing2::findTarget(MyPoint* actual)
{
	// Removing points from the queue until point with higher distance
	// from robot than targetDistance is found.
      
	if (trajectory.empty() == false)
	{
	//   printf("l:x %f y %f ,f: x%f  y%f f\r\n",lastRemoved->x,lastRemoved->y,trajectory.front()->x,trajectory.front()->y);
	while (actual->getDistance(trajectory.front()) < targetDistance)
	{
		lastRemoved = trajectory.front();
		trajectory.pop();
		if(trajectory.empty() == true){
			break;
		}
	}
	}
	if (trajectory.empty() == true)
	{
		target->x = lastRemoved->x;
		target->y = lastRemoved->y;
	}
	else
	{
		//circle - line aproximation
		
		//vector: FRONT - LAST REMOVED
		MyPoint* s = new MyPoint(trajectory.front()->x - lastRemoved->x, trajectory.front()->y - lastRemoved->y);
		
		//vector which will be added to lastRemoved
		MyPoint* v = new MyPoint(0.0,0.0);
		 printf("  actual- l:%f,actual-f:%f ",actual->getDistance(lastRemoved),actual->getDistance(trajectory.front()));  //targetDistance  0.2
		if (actual->getDistance(lastRemoved) >= targetDistance && actual->getDistance(trajectory.front()) >= targetDistance && 
		(lastRemoved->x != trajectory.front()->x || lastRemoved->y != trajectory.front()->y))	//two intersections
		{
		      
			//finding point between intersections
			//calculating line
			double a, b, c, dist;
			a = lastRemoved->y - trajectory.front()->y;
			b = trajectory.front()->x - lastRemoved->x;	
			c = lastRemoved->x * trajectory.front()->y - trajectory.front()->x * lastRemoved->y;
			if (a * a + b * b < 0.001)	//front and lastRemoved are the same
			{
				target->x = lastRemoved->x;
				target->y = lastRemoved->y;
				return;
			}		
			dist = fabs(a*actual->x + b*actual->y + c)/sqrt(a * a + b * b);	//distance between point and line
			double distFromLast;
			distFromLast = sqrt(pow(lastRemoved->getDistance(actual),2.0) - pow(dist,2.0));
			
			v->x = (distFromLast/s->getAbs())*s->x;
			v->y = (distFromLast/s->getAbs())*s->y;
			
			printf("distFromLast %f v:x %f,y:%f     ",distFromLast,v->x,v->y);
			s->x = s->x - v->x;
			s->y = s->y - v->y;
			printf("s:x %f,y:%f     ",s->x,s->y);
		}
		
		for (int i = 1; i < APPROX_ITERATIONS; i++)
		{
			if ((*lastRemoved + s->times(pow(2,-i)) + *v - *actual).getAbs() < targetDistance)
			{
				*v = *v + s->times(pow(2,-i));
			}
			
		}
		target->x = lastRemoved->x + v->x;
		target->y = lastRemoved->y + v->y;
		printf("traget x:%f,y%f\r\n",(*target).x,target->y);
	}
	
	
}

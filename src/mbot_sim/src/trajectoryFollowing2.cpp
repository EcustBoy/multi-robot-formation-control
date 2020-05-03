#include "myPoint.h"
#include "node_trajectoryFollowing2.h"


#define SUBSCRIBER_BUFFER_SIZE 1	//!<Size of buffer for subscriber.

#define PUBLISHER_BUFFER_SIZE 1000	//!<Size of buffer for publisher.

#define TOLERANCE 0.05	//!<Distance from the target point, at which the point will be considered as achieved.

#define SPEED 0.15		//!<Speed of robot.

#define LOOK_AHEAD 0.2	//!<Robot will follow point on trajectory at this distance.

// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"

#define PUBLISHER_TOPIC "/cmd_vel"

// #define SUBSCRIBER_TOPIC "/syros/global_odom"

//#define SUBSCRIBER_TOPIC "odom"
#define SUBSCRIBER_TOPIC "amcl_pose"

/*! \brief Starting node for task: "Trajectory Following 2"
 * 
 * In main function is created Subscribing node, which transmits messages 
 * to NodeTrajectoryFollowing1 object. There are the messages processed and commands 
 * generated.
 */

int main(int argc, char **argv)
{
	//Initialization of node
	ros::init(argc, argv, "trajectoryFollowing2");
	ros::NodeHandle n;
	
	//Creating publisher
	ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
	
	//This trajectory will be used
	std::queue<MyPoint*> trajectory;
     
       

       
	MyPoint* p1 = new MyPoint(0.0, 0.0);
	MyPoint* p2 = new MyPoint(0.0, 2.0);
	MyPoint* p3 = new MyPoint(2.0, 2.0);
	//MyPoint* p4 = new MyPoint(-1.0, 2.0);
	trajectory.push(p1);
	trajectory.push(p2);
	trajectory.push(p3);
	//trajectory.push(p4);
	
	//Creating object, which stores data from sensors and has methods for
	//publishing and subscribing
	NodeTrajectoryFollowing2 *nodeTrajectoryFollowing2 = new NodeTrajectoryFollowing2(pubMessage, trajectory, TOLERANCE, SPEED, LOOK_AHEAD);
	
	//Creating subscriber and publisher
	ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeTrajectoryFollowing2::messageCallback, nodeTrajectoryFollowing2);
	ros::spin();

	return 0;
}

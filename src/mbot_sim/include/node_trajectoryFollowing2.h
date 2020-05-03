#ifndef SR_NODE_TRAJECTORY_FOLLOWING_2
#define SR_NODE_TRAJECTORY_FOLLOWING_2

#include "myPoint.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


/*! \brief Demonstration task: "Trajectory Following 2"
 * 
 * This class controls robot. Robot goes along defined trajectory.
 */
 
 class NodeTrajectoryFollowing2
{
public:
	
	/*! \brief A constructor.
	 * 
	 * @param pub Publisher, which can send commands to robot.
	 * @param followedTrajectory Robot will follow this trajectory.
	 * @param tol Position tolerance [m].
	 * @param sp Default speed of robot.
	 * @param relativeSp If this is set to true, speed will change according to distance from target point.
	 */
	NodeTrajectoryFollowing2(ros::Publisher pub, std::queue<MyPoint*> followedTrajectory, double tol, double sp, double targetDist);
	
	/*! \brief A destructor.
	 */
	~NodeTrajectoryFollowing2();

	/*! \brief This method publishes commands for robot.
	 *
	 * @param angleCommand Angular velocity.
	 * @param speedCommand Velocity.
	 */
	void publishMessage(double angleCommand, double speedCommand);

	/*! \brief This method reads odometry data and processes them to variables.
	 * 
	 * This method saves actual position
	 * 
	 * @param msg Message, which came from robot and contains data from
	 * laser scan.
	 */
	//void messageCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void messageCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	/*! \brief This method calculates angle difference from desired point.
	 * 
	 * @param actual Actual position of robot.
	 * @param angle Actual orientation of robot.
	 */
	double calculateAngle(MyPoint* actual, double angle, double speed);
	
	/*! \brief This method calculates, if robot is close enough from target point.
	 * 
	 * If distance between robot and target point is shorter than 
	 * tolerance, target is accomplished and method returns true.
	 * 
	 * @param actual Actual position of robot.
	 */
	bool closeEnough(MyPoint* actual);
	
	/*! \brief This method calculates speed.
	 * 
	 * If @relativeSpeed is true, robot slows, if it is near it's destination, 
	 * to make more precise moves. Otherwise this method returns @speed.
	 * 
	 * @param actual Actual position of robot.
	 */
	 double calculateSpeed(MyPoint* actual);
	 
	 /*! \brief This method finds target point.
	  * 
	  * Target point is on the trajectory in @targetDistance from actual 
	  * position of robot.
	  * 
	  * @param actual Actual position of robot.
	  */
	 void findTarget(MyPoint* actual);

//variables
	std::queue<MyPoint*> trajectory;	//!<Defined trajectory.
	MyPoint* lastRemoved;	//!<Last point, removed frome the queue.
	MyPoint* target;		//!<Target point, which is actual destination of robot.
	double tolerance;	//!<Distance from destination, which will be tolerated.
	double speed;		//!<Default speed of robot.
	ros::Publisher pubMessage;	//!<Object for publishing messages.
	double targetDistance;	//!< Distance of target point on trajectory. (targetDistance >> tolerance)
};

#endif

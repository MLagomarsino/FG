#include "ros/ros.h"
#include "football_game/ReachGoal.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h" // to publish the velocity


/** @file */
double robot_pos_x; /*!< x coordinate of the robot */
double robot_pos_y; /*!< y coordinate of the robot */
double robot_yaw; /*!< yaw angle of the robot */

bool goalReached = false; /*!< goalReached flag to check if the goal has been reached */
ros::Publisher pub_vel; /*!< publisher of the robot velocity */

/** Service function
	 * @param[in]  req		odometry message, namely current position of robot 
	 * @param[in]  res 		boolean which tells if the goal has been reached 
*/
bool reach_goal(football_game::ReachGoal::Request &req, football_game::ReachGoal::Response &res)
{
	geometry_msgs::Twist vel;
	// distance of the robot from the desired position (along x and y)
	double dist_x, dist_y = 0;

	// if the robot has a certain distance from the goal, move
	if(!goalReached && (fabs(robot_pos_x - req.robot_des.pose.pose.position.x) > 0.05 || fabs(robot_pos_y - req.robot_des.pose.pose.position.y) > 0.05)){
		
		dist_x = robot_pos_x - req.robot_des.pose.pose.position.x;
		dist_y = robot_pos_y - req.robot_des.pose.pose.position.y;
		// std::cout<<"robot pos x\n"<<robot_pos_x<<"\n";
		// std::cout<<"robot pos y\n"<<robot_pos_y<<"\n";		
		
		// linear velocity along x (bounded [-1,+1])
		if (fabs(dist_x) > 0.05){
			vel.linear.x = -0.5*(dist_x); 
			if(vel.linear.x > 1)
				vel.linear.x = 1;
			else if (vel.linear.x < -1)
				vel.linear.x = -1;
		}
		else{	
			vel.linear.x = 0;
		}

		// linear velocity along y (bounded [-1,+1])
		if (fabs(dist_y) > 0.05){
			vel.linear.y = -0.5*(dist_y);
			if(vel.linear.y > 1)
				vel.linear.y = 1;
			else if (vel.linear.y < -1)
				vel.linear.y = -1;
		}
		else{
			vel.linear.y = 0;
		}
	}

	// std::cout<<"robot yaw \n"<<robot_yaw<<"\n";
	
	// if the linear velocities are 0 (goal position reached), start rotating
	if(vel.linear.x == 0 && vel.linear.y == 0){
		geometry_msgs::Quaternion quat = req.robot_des.pose.pose.orientation;
		double robot_des_yaw = asin(2*quat.x*quat.y + 2*quat.z*quat.w);
		std::cout<<"\n--- desired yaw"<<robot_yaw;
		// std::cout<<"robot des yaw \n"<<robot_des_yaw<<"\n";

		// angular velocity along z
		if(fabs(robot_yaw - robot_des_yaw) > 0.01)
			vel.angular.z = -1*(robot_yaw - robot_des_yaw);
		else{
			vel.angular.z = 0;
			goalReached = true;
		}
	}
	
	// if the goal has been reached go ahead and kick the ball
	if(goalReached)
		vel.linear.y = -1;
	
	// std::cout << "linear vel x "<<vel.linear.x<<"\n";
	// std::cout << "linear vel y "<<vel.linear.y<<"\n";
	// std::cout << "angular vel z "<<vel.angular.z<<"\n";
	
	// Publish the velocity
	pub_vel.publish(vel);   
	
	// wait some time, then stop and start looking again for the ball
	if(goalReached){
		std::cout << "Goal reached\n";
		vel.linear.y = 0;
		sleep(2);  
		pub_vel.publish(vel);
		goalReached = false;
		sleep(2); 
	} 
	res.ack = true;	
	return true;
}

/** Callback associated to topic /odom 
 *
 * The function saves the current position and yaw of the robot in global variables
 * @param[in]  msg		odometry message  current position of the robot wrt world frame */
void odomCallback(const nav_msgs::Odometry& msg)
{
	geometry_msgs::Twist vel;
	
	robot_pos_x = msg.pose.pose.position.x;
	robot_pos_y = msg.pose.pose.position.y;
	geometry_msgs::Quaternion quat = msg.pose.pose.orientation;
	robot_yaw = asin(2*quat.x*quat.y + 2*quat.z*quat.w);
	std::cout<<"\n--- current yaw"<<robot_yaw;
}


/** Main function
 *
 * - definition of the publisher for /cmd_vel (topic where the velocities of the robot are published)
 * - definition of the service reach_goal
 * - definition of the subscriber to the /odom topic, where the odometry of the robot is published */
int main(int argc, char ** argv) 
{
	ros::init(argc, argv, "reach_goal");
	ros::NodeHandle n;
	pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::ServiceServer server_reach_goal = n.advertiseService("reach_goal", reach_goal);
	
	// Subscriber to the odom topic in order to get the current robot position
	ros::Subscriber sub_odom = n.subscribe("odom", 1000, odomCallback);

	ros::spin();

	return 0;
}

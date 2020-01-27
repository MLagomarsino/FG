#include "ros/ros.h"
#include "football_game/ReachGoal.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h" // to publish the velocity


float robot_pos_x;
float robot_pos_y;
float robot_pos_z;
float robot_yaw;
bool goalReached = false;
// Publisher of the robot velocity
ros::Publisher pub_vel;

bool reach_goal(football_game::ReachGoal::Request &req, football_game::ReachGoal::Response &res)
{
	geometry_msgs::Twist vel;
	double dist_x, dist_y = 0;
	// Motion along y

	if(!goalReached && (fabs(robot_pos_x - req.robot_des.pose.pose.position.x) > 0.05 || fabs(robot_pos_y - req.robot_des.pose.pose.position.y) > 0.05)){
		
		dist_x = robot_pos_x - req.robot_des.pose.pose.position.x;
		dist_y = robot_pos_y - req.robot_des.pose.pose.position.y;
		std::cout<<"robot pos x\n"<<robot_pos_x<<"\n";
		std::cout<<"robot pos y\n"<<robot_pos_y<<"\n";
		std::cout<<"robot pos z\n"<<robot_pos_z<<"\n";		
		// inf and sup limits to limit robot velocity
		if (fabs(dist_x) > 0.05){
			std::cout<<"heila sono la x\n"<<dist_x<<"\n";
			vel.linear.x = -0.5*(dist_x); 
			if(vel.linear.x>1)
				vel.linear.x=1;
			else if (vel.linear.x<-1)
				vel.linear.x=-1;
		}
		else{	
			std::cout<<"heila sono la x a zero\n"<<dist_x<<"\n";
			vel.linear.x = 0;
		}

		if (fabs(dist_y) > 0.05){
			std::cout<<"heila sono la y\n"<<dist_y<<"\n";
			vel.linear.y = -0.5*(dist_y);
			if(vel.linear.y>1)
				vel.linear.y=1;
			else if (vel.linear.y<-1)
				vel.linear.y=-1;
		}
		else{
			std::cout<<"heila sono la y a zero\n"<<dist_y<<"\n";
			vel.linear.y =0;
		}
	}
	std::cout<<"robot yaw \n"<<robot_yaw<<"\n";
	
	if(vel.linear.x == 0 && vel.linear.y == 0){
		geometry_msgs::Quaternion quat = req.robot_des.pose.pose.orientation;
		float robot_des_yaw = asin(2*quat.x*quat.y + 2*quat.z*quat.w);
		std::cout<<"robot des yaw \n"<<robot_des_yaw<<"\n";
		if(fabs(robot_yaw - robot_des_yaw) > 0.01)
			vel.angular.z = -1*(robot_yaw - robot_des_yaw);
		else{
			vel.angular.z = 0;
			goalReached = true;
		}
	}
	
	if(goalReached)
		vel.linear.y = -1;
	
	// Goal reached 
	std::cout << "linear vel x "<<vel.linear.x<<"\n";
	std::cout << "linear vel y "<<vel.linear.y<<"\n";
	std::cout << "angular vel z "<<vel.angular.z<<"\n";
	
	// Publish the velocity
	pub_vel.publish(vel);   
	
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

/** Callback associated to topic ../odom
 * 
 * According to desired position, the function 
 * - computes the linear velocity of the robot and publishes it on /cmd_vel
 * - if the goal is reached, it notifies it by publishing on /goal_reached
 * @param[in]  msg		odometry message  current position of the robot*/
void odomCallback(const nav_msgs::Odometry& msg)
{
	geometry_msgs::Twist vel;
	
	// Motion along x
	robot_pos_x = msg.pose.pose.position.x;
	robot_pos_y = msg.pose.pose.position.y;
	robot_pos_z = msg.pose.pose.position.z;
	geometry_msgs::Quaternion quat = msg.pose.pose.orientation;
	robot_yaw = asin(2*quat.x*quat.y + 2*quat.z*quat.w);
	
}
	
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

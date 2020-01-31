#include "ros/ros.h"
#include "nav_msgs/Odometry.h" // used for the robot goal position
#include "std_msgs/Bool.h" // for the ack message

#include <string>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "football_game/ReachGoal.h"

#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

using namespace std;

/** @file */
ros::Publisher pub_goal;		/*!< Publisher of the goal of the robot on /goal*/
ros::Subscriber odom_sub;		/*!< Subscriber to /odom*/
nav_msgs::Odometry robot_des;	/*!< Odometry message describing the desired position of robot*/

float x_ball; /*!< x coordinate of the ball */
float y_ball; /*!< y coordinate of the ball */

float x_robot; /*!< x coordinate of the robot wrt the world frame */
float y_robot; /*!< y coordinate of the robot  wrt the world frame */
geometry_msgs::Quaternion orientation_robot; /*!< orientation of the robot wrt the world frame */
float yaw;  /*!< yaw angle between the ball and the football goal */

tf::Transform world2ball; /*!< computed transform between ball and world frame*/

/** @brief Class to implement ball tracking
 * 
 *  The class computes :
 *	*	position of the ball wrt the world frame;
 *	*	yaw angle between football goal and the ball, useful to kick the ball.
 */
class BallPositionWorld {
public:
    /**
     * Transformation from world to ball frame
     */
    tf::TransformListener listener;
     
    /** Handler:
     * - subscribe to /ball_coord to read the position of the ball
     * - subscribe to /odom topic to read robot odometry wrt the world frame
     */
    BallPositionWorld()
    {
        ball_pos_sub = nh.subscribe("/ball_coord", 10, &BallPositionWorld::ballPosCallback, this);
		odom_sub = nh.subscribe("/odom", 1000, &BallPositionWorld::odomCallback, this);
    }

	/** Callback associated to topic /odom
	 * @param[in]  msg		odometry message, namely current position of robot wrt the world frame
	 */
	void odomCallback(const nav_msgs::Odometry& msg)
	{
		x_robot = msg.pose.pose.position.x; /**< x coordinate of the robot */
		y_robot = msg.pose.pose.position.y; /**< y coordinate of the robot */
		orientation_robot = msg.pose.pose.orientation; /**< orientation of the robot*/
	}
    /** 
     * Callback function
     * @param[in]  ball_pos position of the ball wrt the camera -(if not published from terminal)-
     */
    void ballPosCallback(const geometry_msgs::Point &ball_pos)
	{
		x_ball = ball_pos.x;
		y_ball = ball_pos.z;

		try{    
			listener.waitForTransform("football_goal_frame","world_frame",  ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform("football_goal_frame","world_frame", ros::Time(0), t_football_goal2world);
			
			try{    
				listener.waitForTransform("world_frame", "robot_frame", ros::Time(0), ros::Duration(10.0) );
				listener.lookupTransform("world_frame", "robot_frame",  ros::Time(0), t_world2robot);

				try{    
					listener.waitForTransform("robot_frame", "ball_frame", ros::Time(0), ros::Duration(10.0) );
					listener.lookupTransform("robot_frame", "ball_frame",  ros::Time(0), t_robot2ball);
					tf::Transform robot2ball(t_robot2ball.getBasis(), t_robot2ball.getOrigin());
					//world2ball = t_world2robot * robot2ball;
					world2ball = robot2ball;
					
					tf::Transform football_goal2ball = t_football_goal2world * world2ball;
					tf::Vector3 ball_football_direction;
					ball_football_direction = football_goal2ball.getOrigin();
					
					// compute yaw angle between football goal and the ball, usefull to kick the ball
					yaw = atan((ball_football_direction.getY())/(ball_football_direction.getX()));
					std::cout<<"\nx_ball: "<<ball_football_direction.getX();
					std::cout<<"\ny_ball: "<<ball_football_direction.getY();
					
				}
				catch (tf::TransformException &ex){
					ROS_WARN("World to robot transform unavailable %s", ex.what());
				}
			}
			catch (tf::TransformException &ex){
				ROS_WARN("World to robot transform unavailable %s", ex.what());
			}
		}
		catch (tf::TransformException &ex){
			ROS_WARN("Robot to ball transform unavailable %s", ex.what());
		}	
}

protected:
	tf::StampedTransform t_world2robot;	/**< Transformation from world_frame to robot_frame*/
	tf::StampedTransform t_football_goal2world;	/**< Transformation from football_goal_frame to world_frame*/
	tf::StampedTransform t_robot2ball; /**< Transformation from robot_frame to ball_frame*/

    ros::NodeHandle nh;		/**< Node Handler */
    ros::Subscriber ball_pos_sub; /**< Subscriber to /ball_coord */
};

/** The function computes the desired position and orientation of the robot to kick the ball.
 * @param[out]  robot_des	odometry message specifing the desired robot position
 */
nav_msgs::Odometry compute_plan()
{
	tf::Vector3 ball_point(x_ball, y_ball, 0);
	tf::Vector3 ball_wrt_world;
	// ball wrt world frame
	// ball_wrt_world = t_world2robot * ball_point; // ONLY WHEN USING THE REAL ROBOT (the position of the ball is published wrt robot frame) 
	ball_wrt_world = ball_point; // ONLY WHEN position of the ball is published from terminal (wrt world frame) 

	double r = 1; // radius from the ball
	// desired position of the robot
	robot_des.pose.pose.position.x = ball_wrt_world.getX() + r*cos(yaw);
	robot_des.pose.pose.position.y = ball_wrt_world.getY() + r*sin(yaw);
	robot_des.pose.pose.position.z = 0;
	// std::cout<<"robot des pos x "<<robot_des.pose.pose.position.x<<"\n";
	// std::cout<<"robot des pos y "<<robot_des.pose.pose.position.y <<"\n";

	tf::Quaternion quat;
	quat.setRPY(0, 0, -yaw-1.570796);
	std::cout<<"\ndesired orientation "<<(-yaw-1.570796);
	// desired orientation of the robot
	robot_des.pose.pose.orientation.x = quat.x();
	robot_des.pose.pose.orientation.y = quat.y();
	robot_des.pose.pose.orientation.z = quat.z();
	robot_des.pose.pose.orientation.w = quat.w();

	return robot_des;
}

/**
 * Main function:
 *
 * - definition of the publisher of the /goal topic where the goal position of the robot is published
 * - declare and call the reach_goal service to move toward the desired position and orientation
 * @param[in]  r_x			x coordinate of the robot
 * @param[in]  r_y			y coordinate of the robot
 */
int main(int argc, char ** argv) 
{
	ros::init(argc, argv, "compute_goal");
	ros::NodeHandle n;
	ros::ServiceClient client_reach_goal;
	BallPositionWorld handler;
	bool result = false;
	
	// Declare variables for the initial positions of the robot
	int r_start_x, r_start_y;

	// Get the initial position of the robot from the parameter server
	n.getParam("r_x", r_start_x);
	n.getParam("r_y", r_start_y);

	// Publisher of the goal position
	pub_goal = n.advertise<nav_msgs::Odometry>("/goal", 1);
	
	client_reach_goal = n.serviceClient<football_game::ReachGoal>("/reach_goal");
    	
	ros::Rate r(10000);

	while(ros::ok()){
		// if the ball has been detected, compute desired position and call the service
		if (x_ball != 0 && y_ball != 0){
			// call the compute goal function which returns the desired position of the robot
			robot_des = compute_plan();
			football_game::ReachGoal srv;
			// the request to the service is the desired position
			srv.request.robot_des = robot_des;
			result = client_reach_goal.call(srv);
		}
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

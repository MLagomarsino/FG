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

ros::Publisher pub_goal; /*!< Publisher of the goal of the robot on /goal*/
ros::Subscriber odom_sub; /*!< Subscriber to /odom*/
nav_msgs::Odometry robot_des; /*!< Odometry message describing the desired position of robot*/

geometry_msgs::Vector3 ball_football_direction; /**< direction between ball and football goal */

float x_ball; /**< X coordinate of the ball */
float y_ball; /**< Y coordinate of the ball */

float x_robot; /**< X coordinate of the robot */
float y_robot; /**< Y coordinate of the robot */
geometry_msgs::Quaternion orientation_robot;
float yaw;

tf::Transform world2ball;

class BallPositionWorld {
public:
    /**
     * Transformation from Base Frame to Camera Frame
     */
    tf::TransformListener listener;
     
    /** Handler:
     * - subscribe to /camera/pcl_filtered to get filtered point cloud sent by pcl_filter
     * - publish odometry data on /odometry/baxter/center_of_mass
     * - publish a frame with origin in the computed center of mass and the same orientation of the Kinect 
     */
    BallPositionWorld()
    {
        ball_pos_sub = nh.subscribe("/ball_coord", 10, &BallPositionWorld::ballPosCallback, this);
		odom_sub = nh.subscribe("/odom", 1000, &BallPositionWorld::odomCallback, this);
    }

	/** Callback associated to topic /odom
	 * @param[in]  msg		odometry message, namely current position of robot */
	void odomCallback(const nav_msgs::Odometry& msg)
	{
		x_robot = msg.pose.pose.position.x; /**< X coordinate of the robot */
		y_robot = msg.pose.pose.position.y; /**< Y coordinate of the robot */
		orientation_robot = msg.pose.pose.orientation; /**< Orientation of the robot with respect to the world frame*/
	}
    /** 
     * Callback function
     * @param[in]  input	point cloud data from /camera/pcl_filtered
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
					world2ball = t_world2robot * robot2ball;

					tf::Transform football_goal2ball = t_football_goal2world * world2ball;
					tf::Vector3 ball_football_direction;
					ball_football_direction = football_goal2ball.getOrigin();
					
					tf::Quaternion ball_football_quaternion;
					ball_football_quaternion = football_goal2ball.getRotation();
					yaw = asin(2*ball_football_quaternion.x()*ball_football_quaternion.y() + 2*ball_football_quaternion.z()*ball_football_quaternion.w());

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
	tf::StampedTransform t_world2robot;	/**< Transformation from world frame to camera link frame*/
	tf::StampedTransform t_football_goal2world;	/**< Transformation from camera link frame to camera depth optical frame*/
	tf::StampedTransform t_robot2ball;

    ros::NodeHandle nh;		/**< Node Handler */
    ros::Subscriber ball_pos_sub; /**< Subscriber to /ball_coord */
};

/** The function:
 * - acquires the initial position of the robot (robot_des of the previous loop)
 * @param[in]  robot_des	odometry message specifing the initial position of the robot (desired position of the previous loop)
 * @param[out]  robot_des	odometry message specifing the desired robot position
 */
nav_msgs::Odometry compute_plan()
{
	tf::Vector3 ball_point(x_ball,y_ball,0);
	tf::Vector3 ball_wrt_world;
	// ball wrt world frame
	ball_wrt_world = world2ball * ball_point;
	int r = 300; // radius from the ball
	robot_des.pose.pose.position.x = ball_wrt_world.getX() + r*cos(yaw);
	robot_des.pose.pose.position.y = ball_wrt_world.getY() + r*sin(yaw);	
	robot_des.pose.pose.position.z = 0;
	tf::Quaternion quat;
	quat.setRPY(0,0,yaw);
	robot_des.pose.pose.orientation.x = quat.x();
	robot_des.pose.pose.orientation.y = quat.y();
	robot_des.pose.pose.orientation.z = quat.z();
	robot_des.pose.pose.orientation.w = quat.w();

	return robot_des;
}

/**
 * Main function:
 * - subscribe to /odom to know the current position of the robot 
 * - subscribe to /ball_coord to know the position of the ball wrt the robot
 * - compute and publish on /goal the goal position of the robot
 * - 
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
    	
	ros::Rate r(10000);

	while(ros::ok()){

		if (x_ball != 0 && y_ball != 0){
			robot_des = compute_plan();
			football_game::ReachGoal srv;
			srv.request.robot_des = robot_des;
			result = client_reach_goal.call(srv);
		}
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

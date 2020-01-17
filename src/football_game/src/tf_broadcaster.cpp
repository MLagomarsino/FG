#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <math.h>


double x_robot;
double y_robot;
geometry_msgs::Quaternion orientation_robot;

double x_ball;
double y_ball;

/** @brief Class to publish periodically the transformation between the ball frame and the world frame
 */
class TF_Broadcaster{
    /** TF_Broadcaster:
     * - subscribe to /ball_coord to know the current position of the ball
     */
    public:
    TF_Broadcaster(){
        ball_pos_sub = nh.subscribe("ball_coord", 2, &TF_Broadcaster::ball_posCB, this);
		odom_sub = nh.subscribe("odom", 2, &TF_Broadcaster::odomCB, this);
    }
    /**
     * Position callback function
     * acquires the current position of the ball
     * @param[in]  ball_pos	current position of the ball
     */
    void ball_posCB(const geometry_msgs::Point& ball_pos){
    	x_ball = ball_pos.x;
		y_ball = ball_pos.z;
    }

    /**
     * Position callback function
     * acquires the current position of the robot
     * @param[in]  robot_pos	current position of the ball
     */
    void odomCB(const nav_msgs::Odometry& robot_pos){
    	x_robot = robot_pos.pose.pose.position.x;
		y_robot = robot_pos.pose.pose.position.y;
		orientation_robot = robot_pos.pose.pose.orientation;
    }
	
protected:
    ros::NodeHandle nh; /**< Node Handle */
    ros::Subscriber ball_pos_sub;  /**< Subscriber to /ball_cord */
    ros::Subscriber odom_sub;  /**< Subscriber to /odom */
};

    
/**
 * Main function: 
 * a TF broadcaster is created in order to add to the tree of frames:
 * 	- The world frame: in correspondence of the control board and with the same orientation of the Kinect;
 * 	- two frames for images in rviz (robot LCD and FSM schema).
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_ball_broadcaster");
	ros::NodeHandle n("~");

	TF_Broadcaster tf_broadcaster;

	ros::Rate r(10000);
	tf::TransformBroadcaster broadcaster;

	// Add frames to the tree (robot frame and ball frame)
	tf::Transform world2robot;
	world2robot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	
	tf::Transform robot2ball;
	robot2ball.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    robot2ball.setRotation(tf::Quaternion(1,0,0,0));

	tf::Transform football_goal2world;
	football_goal2world.setOrigin(tf::Vector3(0.0, -120.0, 0.0)); //240x360 campo
	football_goal2world.setRotation(tf::Quaternion(1,0,0,0));

	while(ros::ok()){
		
		world2robot.setOrigin(tf::Vector3(x_robot, y_robot, 0.0));
		world2robot.setRotation(tf::Quaternion(orientation_robot.x,orientation_robot.y,orientation_robot.z, orientation_robot.w));

		robot2ball.setOrigin(tf::Vector3(x_ball, y_ball, 0.0));
		
		broadcaster.sendTransform(tf::StampedTransform(world2robot, ros::Time::now(), "world_frame", "robot_frame"));
		broadcaster.sendTransform(tf::StampedTransform(robot2ball, ros::Time::now(), "robot_frame", "ball_frame"));
		broadcaster.sendTransform(tf::StampedTransform(football_goal2world, ros::Time::now(), "world_frame", "football_goal_frame"));
				
		ros::spinOnce();
		r.sleep();
	}

    return 0;
}

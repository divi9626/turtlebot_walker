#include "../include/walker.hpp"

Walker::Walker(ros::NodeHandle n)
 : move_pub(n.advertise<geometry_msgs::Twist>("/cmd_vel",10)),
   laser_sub(n.subscribe("/scan", 10, &Walker::laserCallBack, this)){
   	twist.linear.x = 0;
   	twist.linear.z = 0;
   	move_pub.publish(twist);
   	ROS_INFO("Walker node initialized");
   };

 Walker::~Walker(){
 	ROS_INFO("Walker_node is shutting down");
 };

 void Walker::laserCallBack(const sensor_msgs::LaserScan::ConstPtr &scan){
 	bool flag_check = false;
 	if (scan->ranges[0] <= 0.6 || scan->ranges[30] <= 0.6 || scan->ranges[330] <= 0.6){
 		flag_check = true;
 	};

 	if (flag_check){
 		twist.linear.x = 0;
 		twist.angular.z = 0.8;
 		move_pub.publish(twist);
 		ROS_WARN("Obstacle ahead, taking a turn");
 	}
 	if (flag_check == false) {
 		twist.linear.x = 0.5;
 		twist.angular.z = 0.0;
 		move_pub.publish(twist);
 		ROS_INFO("No Obstacle ahead, moving forward");
 	};
 };
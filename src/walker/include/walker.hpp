#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

class Walker {
private:
	geometry_msgs::Twist twist;
	ros::Publisher move_pub;
	ros::Subscriber laser_sub;
public:
	Walker(ros::NodeHandle n);
	~Walker();
	void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
};
#endif // include Walker.hpp
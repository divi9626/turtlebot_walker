#include <ros/ros.h>
#include "../include/walker.hpp"

int main(int argc, char** argv) {
  //> Initializing the node.
  ros::init(argc, argv, "walker");

  //> Creating Node handle.
  ros::NodeHandle n;

  //> Creating Walker walk object.
  Walker walk(n);

  //> Waiting for scan msg call back.
  ros::spin();

  return 0;
}

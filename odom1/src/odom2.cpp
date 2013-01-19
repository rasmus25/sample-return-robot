#include "ros/ros.h"
#include "gazebo_msgs/GetJointProperties.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jointprop");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

  gazebo_msgs::GetJointProperties srv;
  
  if (client.call(srv))
  {
    ROS_INFO("Pos: %ld", (long int)srv.response.position[0]);
  }
  else
  {
    ROS_ERROR("Failed to call service jointprop");
    return 1;
  }

  return 0;
}
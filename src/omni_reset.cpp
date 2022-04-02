#include "ros/ros.h"
#include "omni-robot/omni_reset.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_client");
  if (argc != 2)
  {
    ROS_INFO("usage: reset_client new_count");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pub_sub::Reset>("reset");
  pub_sub::Reset srv;
  srv.request.new_count = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Old count: %ld", (long int)srv.response.old_count);
  }
  else
  {
    ROS_ERROR("Failed to call service reset");
    return 1;
  }

  return 0;
}
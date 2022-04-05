#include "ros/ros.h"
#include "omni_robot/omni_reset.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "omni_reset");

  if (argc != 2)
  {
    ROS_INFO("usage: reset_client new_count");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<omni_robot::omni_reset>("reset");
  omni_robot::omni_reset srv;

  srv.request.x_new = atoll(argv[1]);
  srv.request.y_new = atoll(argv[2]);
  srv.request.theta_new = atoll(argv[3]);

  if (client.call(srv))
  {
    ROS_INFO("Old odometry: [x=%.4f, y=%.4f, theta=%.4f]", srv.response.x_old, srv.response.y_old, srv.response.theta_old);
  }
  else
  {
    ROS_ERROR("Failed to call service reset");
    return 1;
  }

  return 0;
}
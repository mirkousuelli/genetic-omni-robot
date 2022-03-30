#include "car_traj_ctrl/test_car_eight.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  test_car_eight test_car_eight_node;
   
  test_car_eight_node.Prepare();
  
  test_car_eight_node.RunPeriodically(test_car_eight_node.RunPeriod);
  
  test_car_eight_node.Shutdown();
  
  return (0);
}


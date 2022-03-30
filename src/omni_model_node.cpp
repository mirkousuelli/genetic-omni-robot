#include "car_traj_ctrl/car_traj_ctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  car_traj_ctrl car_traj_ctrl_node;
   
  car_traj_ctrl_node.Prepare();
  
  car_traj_ctrl_node.RunPeriodically();
  
  car_traj_ctrl_node.Shutdown();
  
  return (0);
}


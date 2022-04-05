#include "omni_robot/omni_model.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  omni_model omni_model_node;
   
  omni_model_node.Prepare();
  omni_model_node.RunPeriodically();
  omni_model_node.Shutdown();
  
  return (0);
}


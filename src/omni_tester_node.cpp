#include "omni-robot/omni_tester.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  omni_tester omni_tester_node;
   
  omni_tester_node.Prepare();
  omni_tester_node.RunPeriodically(omni_tester_node.RunPeriod);
  omni_tester_node.Shutdown();
  
  return (0);
}


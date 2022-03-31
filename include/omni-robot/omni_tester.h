#ifndef OMNI_TESTER_H_
#define OMNI_TESTER_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

#define PI 3.14159265358979323846

#define NAME_OF_THIS_NODE "omni_tester"


class omni_tester
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber bar_subscriber;
    ros::Publisher foo_publisher;
    
    /* Node periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    double dt;  // integration step
    double x;  // x position
    double y;  // y position
    double theta;  // heading orientation
    double vel_x;  // x velocity
    double vel_y;  // y velocity
    double vel_theta;  // theta velocity

    /* ROS topic callbacks */
    void bar_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* OMNI_TESTER_H_ */

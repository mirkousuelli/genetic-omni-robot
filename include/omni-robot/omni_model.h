#ifndef OMNI_MODEL_H_
#define OMNI_MODEL_H_

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>

#include "omni_odometry.h"

#define NAME_OF_THIS_NODE "omni_model"


class omni_model
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber foo_subscriber;
    ros::Publisher bar_publisher;
    ros::Publisher clock_publisher;

    /* Parameters from ROS parameter server */
    double dt;  // integration step
    double x0;  // x position
    double y0;  // y position
    double theta0;  // heading orientation
    double r;  // Wheel radius
    double l;  // Wheel position along x
    double w;  // Wheel position along y
    double T;  // Gear ratio  

    /* ROS topic callbacks */
    void foo_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    omni_odometry* simulator;

  public:

    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif /* OMNI_MODEL_H_ */

#ifndef OMNI_MODEL_H_
#define OMNI_MODEL_H_

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>

#include "omni_odometry.h"

#define NAME_OF_THIS_NODE "omni_model"


class omni_model
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleCommand_subscriber;
    ros::Publisher vehicleState_publisher;
    ros::Publisher vehicleHeading_publisher;
    ros::Publisher vehiclePosition_publisher;
    ros::Publisher clock_publisher;

    /* Parameters from ROS parameter server */
    double dt;
    double L;
    double x0, y0, theta0, phi0;
    double eps;
    double xp, yp;

    /* ROS topic callbacks */
    void vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    odometry_omni_robot* simulator;

  public:

    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif /* OMNI_MODEL_H_ */

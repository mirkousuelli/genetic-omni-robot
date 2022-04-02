#ifndef OMNI_MODEL_H_
#define OMNI_MODEL_H_

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>

#include "omni_odometry.h"

#define NAME_OF_THIS_NODE "omni_model"
#define WHEELS 4


class omni_model
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber WheelStates_sub;
    ros::Subscriber RobotPose_sub;
    ros::Publisher CmdVel_pub;

    //ros::Publisher bar_publisher;
    //ros::Publisher clock_publisher;

    /* Parameters from ROS parameter server */
    double dt;  // integration step
    double x;  // x position
    double y;  // y position
    double theta;  // heading orientation
    double r;  // Wheel radius
    double l;  // Wheel position along x
    double w;  // Wheel position along y
    double T;  // Gear ratio
    double lw; // l + w : wheel position sum
    double inv_lw; // l + w : wheel position sum
    double ticks[WHEELS];  // ticks for each wheel
    double rpms[WHEELS];  // rotation per minutes for each wheel
    geometry_msgs::TwistStamped cmd_vel_msg; // linear and angular velocity
    double lin_vel;  // linear velocity
    double ang_vel;  // angular velocity
    ros::Time prev;  // ROS time at the previous time
    ros::Time curr;  // ROS time at the current time

    /* ROS topic callbacks */
    void WheelStates_MessageCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void RobotPose_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

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

#ifndef OMNI_MODEL_H_
#define OMNI_MODEL_H_

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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
    ros::Publisher Odom_pub;

    //ros::Publisher bar_publisher;
    //ros::Publisher clock_publisher;

    /* Parameters from ROS parameter server */
    double dt;  // integration step
    double x;  // x position
    double y;  // y position
    double z;  // z position
    double theta;  // heading orientation
    double r;  // Wheel radius
    double l;  // Wheel position along x
    double w;  // Wheel position along y
    double T;  // Gear ratio
    double prev_tick[WHEELS];  // ticks for each wheel (previous time)
    double curr_tick[WHEELS];  // ticks for each wheel (current time)
    double rpm[WHEELS];  // rotation per minutes for each wheel
    double u_wheel[WHEELS];  // rpms computed from ticks
    geometry_msgs::TwistStamped cmd_vel_msg; // linear and angular velocity message
    nav_msgs::Odometry odom_msg;  // odometry message
    double lin_vel_x; // linear velocity on x
    double lin_vel_y; // linear velocity on y
    double lin_vel;  // linear velocity overall
    double ang_vel;  // angular velocity
    ros::Time prev_time;  // ROS time at the previous time
    ros::Time curr_time;  // ROS time at the current time
    tf2_ros::TransformBroadcaster br;  // TF broadcaster
    geometry_msgs::TransformStamped transformStamped;  // TF message

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

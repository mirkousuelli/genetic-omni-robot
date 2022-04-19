/* 1st Project: 4-Wheels Omnidirectionl Robot Odometry
 * Authors: Alessandro Restifo and Mirko Usuelli
 * Course: Robotics 2022, Politecnico di Milano
 */
#ifndef OMNI_TESTER_H_
#define OMNI_TESTER_H_

#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "omni_robot/omni_msg.h"

#define NAME_OF_THIS_NODE "omni_tester"
# define WHEELS 4


class omni_tester
{
  /* Support class which fullfills the following goal:
   * (1) --- (see class "omni_model")
   * (2) Inverse Kinematic
   * (3) --- (see class "omni_model")
   * (4) --- (see class "omni_model")
   */
  private: 
    /* Node handler */
    ros::NodeHandle Handle;

    /* ROS topics */

    // subscribers
    ros::Subscriber CmdVel_sub;

    // publishers
    ros::Publisher WheelsRpm_pub;
    
    /* Node periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    double r;  // Wheel radius
    double l;  // Wheel position along x
    double w;  // Wheel position along y
    double lin_vel_x;  // linear velocity x
    double lin_vel_y;  // linear velocity y
    double ang_vel;  // angular velocity
    ros::Time curr_time;  // ROS time at the current time
    double u_wheel[WHEELS];  // actuation command on wheel
    omni_robot::omni_msg wheels_rpm_msg;  // wheels RPM message

    /* ROS topic callbacks */
    void CmdVel_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  public:
    double RunPeriod;

    /* Node lifecycle */
    void Prepare(void);
    void RunPeriodically(float Period);
    void Shutdown(void);

};

#endif /* OMNI_TESTER_H_ */

#ifndef OMNI_TESTER_H_
#define OMNI_TESTER_H_

#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#define PI 3.14159265358979323846

#define NAME_OF_THIS_NODE "omni_tester"
# define WHEELS 4


class omni_tester
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber CmdVel_sub;
    ros::Publisher WheelsRpm_pub;

    //rosbag::Bag bag;
    
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

    /* ROS topic callbacks */
    void CmdVel_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* OMNI_TESTER_H_ */

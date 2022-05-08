/* 1st Project: 4-Wheels Omnidirectionl Robot Odometry
 * Authors: Alessandro Restifo and Mirko Usuelli
 * Course: Robotics 2022, Politecnico di Milano
 */
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
#include "omni_robot/omni_msg.h"
#include "omni_robot/omni_reset.h"
#include <omni_robot/parametersConfig.h>

#define NAME_OF_THIS_NODE "omni_model"
#define WHEELS 4

enum OdometryConfig{
  /* Enumerative used to distinguish the integration method
   * for the odometry task.
   */
  EULER = 0,
  RUNGE_KUTTA = 1
};


class omni_model
{
  /* Main class where the bag is processed and the following goals are
   * achieved:
   * (1) Direct Kinematic
   * (2) --- (see class "omni_tester")
   * (3) Odometry reset service
   * (4) Dynamic configuration server for the integration method
   */
  private: 
    /* Node handler */
    ros::NodeHandle Handle;

    /* ROS topics */

    // subscribers
    ros::Subscriber WheelStates_sub;
    ros::Subscriber RobotPose_sub;

    // publishers
    ros::Publisher CmdVel_pub;
    ros::Publisher Odom_pub;
    ros::Publisher WheelsRpm_pub;

    /* ROS services */
    ros::ServiceServer Reset_srv;

    /* Dynamic configuration server */
    dynamic_reconfigure::Server<omni_robot::parametersConfig> dynServer;
    dynamic_reconfigure::Server<omni_robot::parametersConfig>::CallbackType f;

    /* Parameters */
    double dt;  // integration step
    double x;  // x position
    double y;  // y position
    double z;  // z position
    double theta;  // heading orientation
    double r;  // Wheel radius
    double l;  // Wheel position along x
    double w;  // Wheel position along y
    double T;  // Gear ratio
    double N;  // Tick count per wheel revolution (wheel encoder)
    bool COLD_START;
    double delta_ticks;
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
    omni_robot::omni_msg wheels_rpm_msg;  // wheels RPM message
    int odom_method;  // dynamic configuration

    /* ROS topic callbacks */
    void WheelStates_MessageCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void RobotPose_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /* ROS service callbacks */
    bool reset_callback(omni_robot::omni_reset::Request &req, omni_robot::omni_reset::Response &res);

    /* ROS dynamic configuration server callbacks */
    void odom_callback(omni_robot::parametersConfig &config, uint32_t level);

    /* Node periodic task */
    void PeriodicTask(void);

  public:

    /* Node lifecycle */
    void Prepare(void);
    void RunPeriodically(void);
    void Shutdown(void);

};

#endif /* OMNI_MODEL_H_ */

#include "omni_robot/omni_tester.h"
#include <boost/filesystem.hpp>

//namespace fs = boost::filesystem;
const bool DEBUG = true;

void omni_tester::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    /* Initialize node state */
    FullParamName = ros::this_node::getName()+"/r";
    if (false == Handle.getParam(FullParamName, r))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/w";
    if (false == Handle.getParam(FullParamName, w))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    /* ROS topics */
    CmdVel_sub = Handle.subscribe("/cmd_vel", 1000, &omni_tester::CmdVel_MessageCallback, this);
    WheelsRpm_pub = Handle.advertise<omni_robot::omni_msg>("wheels_rpm", 1000);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void omni_tester::CmdVel_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    //curr_time = msg->header.stamp;
    lin_vel_x = msg->twist.linear.x;
    lin_vel_y = msg->twist.linear.y;
    ang_vel = msg->twist.angular.z;
    if (DEBUG) {
        //ROS_INFO("[TIME] *received* current time: %.4f", curr_time.toSec());
        ROS_INFO("[FORWARD-KIN] *received* linear velocity x: %.4f", lin_vel_x);
        ROS_INFO("[FORWARD-KIN] *received* linear velocity y: %.4f", lin_vel_y);
        ROS_INFO("[FORWARD-KIN] *received* angular velocity z: %.4f", ang_vel);
    }

    u_wheel[0] = (1 / r) * (lin_vel_x - lin_vel_y - (l + w) * ang_vel);
    u_wheel[1] = (1 / r) * (lin_vel_x + lin_vel_y + (l + w) * ang_vel);
    u_wheel[2] = (1 / r) * (lin_vel_x + lin_vel_y - (l + w) * ang_vel);
    u_wheel[3] = (1 / r) * (lin_vel_x - lin_vel_y + (l + w) * ang_vel);

    wheels_rpm_msg.rpm_fl = u_wheel[0];
    wheels_rpm_msg.rpm_fr = u_wheel[1];
    wheels_rpm_msg.rpm_rr = u_wheel[2];
    wheels_rpm_msg.rpm_rl = u_wheel[3];
    WheelsRpm_pub.publish(wheels_rpm_msg);

    if (DEBUG) {
        for (int i = 0; i < WHEELS; i++) {
            ROS_INFO("[WHEEL-%i] *computed* u: %.4f", i + 1, u_wheel[i]);
        }
    }

    if (DEBUG) {
        std::cout << std::endl;
    }
}

void omni_tester::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void omni_tester::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void omni_tester::PeriodicTask(void)
{
    ;
}

#include "omni-robot/omni_model.h"
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <math.h>

const bool DEBUG = false;


void omni_model::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    /* loading parameter from the yalm server */
    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/z0";
    if (false == Handle.getParam(FullParamName, z))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta0";
    if (false == Handle.getParam(FullParamName, theta))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/r";
    if (false == Handle.getParam(FullParamName, r))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/w";
    if (false == Handle.getParam(FullParamName, w))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    WheelStates_sub = Handle.subscribe("/wheel_states", 1000, &omni_model::WheelStates_MessageCallback, this);
    RobotPose_sub = Handle.subscribe("/robot/pose", 1000, &omni_model::RobotPose_MessageCallback, this);
    CmdVel_pub = Handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
    Odom_pub = Handle.advertise<nav_msgs::Odometry>("/odom", 1000);

    /* Create simulator class */
    simulator = new omni_odometry(dt);

    /* Initialize simulator class */
    simulator->setInitialState(x, y, theta);
    simulator->setOmniParams(r, l, w, T);

    ros::Time::init();
    prev_time = ros::Time::now();
    curr_time = ros::Time::now();

    for (int i = 0; i < WHEELS; i++) {
        prev_tick[i] = 0.0;
        curr_tick[i] = 0.0;
        u_wheel[i] = 0.0;
    }

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void omni_model::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait for other nodes to start
    sleep(1.0);

    while (ros::ok())
    {
        PeriodicTask();
        ros::spinOnce();
        usleep(1000);
    }
}

void omni_model::Shutdown(void)
{
    // Delete odometry object
    delete simulator;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void omni_model::RobotPose_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    /*ROS_INFO("[POSE] x: %.4f", msg->pose.position.x);
    ROS_INFO("[POSE] y: %.4f", msg->pose.position.y);
    ROS_INFO("[POSE] z: %.4f", msg->pose.position.z);*/
    /*tf::Pose pose;
    tf::poseMsgToTF(msg->pose, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());
    ROS_INFO("[POSE] YAW: %.4f", yaw_angle);*/
}

void omni_model::WheelStates_MessageCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    /* node time update */
    curr_time = ros::Time::now();
    dt = (curr_time - prev_time).toSec();
    if (DEBUG) {
        ROS_INFO("[TIME] Previous time: %.4f", prev_time.toSec());
        ROS_INFO("[TIME] Current time: %.4f", curr_time.toSec());
        ROS_INFO("[TIME] Sampling Ts: %.4f", dt);
    }
    prev_time = curr_time;

    for (int i = 0; i < WHEELS; i++) {
        curr_tick[i] = msg->position.at(i);
        if (true) {
            // ticks to rpm (more accurate)
            u_wheel[i] = (curr_tick[i] - prev_tick[i]) / 360 * 4096 * T;
        } else {
            // rpm (noisy)
            u_wheel[i] = msg->velocity.at(i);
        }
        prev_tick[i] = curr_tick[i];

        if (true)
            ROS_INFO("[WHEEL-%i] u: %.4f", i + 1, u_wheel[i]);
    }

    /* forward kinematic */
    lin_vel_x = (r / 4) * (u_wheel[0] + u_wheel[1] + u_wheel[2] + u_wheel[3]);
    lin_vel_y = (r / 4) * (-u_wheel[0] + u_wheel[1] + u_wheel[2] - u_wheel[3]);
    ang_vel = (r / 4) * (1 / (l + w)) * (-u_wheel[0] + u_wheel[1] - u_wheel[2] + u_wheel[3]);
    if (true) {
        ROS_INFO("[FORWARD-KIN] linear velocity x: %.4f", lin_vel_x);
        ROS_INFO("[FORWARD-KIN] linear velocity y: %.4f", lin_vel_y);
        ROS_INFO("[FORWARD-KIN] angular velocity z: %.4f", ang_vel);
    }

    // linear and angular velocities published into topic '/cmd_vel'
    cmd_vel_msg.twist.linear.x = lin_vel_x;
    cmd_vel_msg.twist.linear.y = lin_vel_y;
    cmd_vel_msg.twist.linear.z = 0.0;
    cmd_vel_msg.twist.angular.x = 0.0;
    cmd_vel_msg.twist.angular.y = 0.0;
    cmd_vel_msg.twist.angular.z = ang_vel;
    CmdVel_pub.publish(cmd_vel_msg);

    /* odometry */
    double delta_x = lin_vel_x * dt;
    double delta_y = lin_vel_y * dt;
    double delta_theta = ang_vel * dt;
    if (false) {
        // Euler
        x += delta_x * std::cos(theta) - delta_y * std::sin(theta);
        y += delta_x * std::sin(theta) + delta_y * std::cos(theta);
    } else {
        // Runge-Kutta
        x += delta_x * std::cos(theta + ang_vel * dt / 2) - delta_y * std::sin(theta + ang_vel * dt / 2);
        y += delta_x * std::sin(theta + ang_vel * dt / 2) + delta_y * std::cos(theta + ang_vel * dt / 2);
    }
    theta += delta_theta;
    if (DEBUG) {
        ROS_INFO("[ODOM] x: %.4f", x);
        ROS_INFO("[ODOM] y: %.4f", y);
        ROS_INFO("[ODOM] theta: %.4f", theta);
    }

    // odometry position published into topic '/odom'
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";
    odom_msg.header.stamp = curr_time;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    Odom_pub.publish(odom_msg);

    // odometry position sent to TF
    transformStamped.header.stamp = curr_time;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);

    if (true) {
        std::cout << std::endl;
    }
}

void omni_model::PeriodicTask(void)
{
    /* Integrate the model */
    //simulator->integrate();

    /* Extract measurement from simulator */
    //double x, y, theta;

    //double u_wheels[WHEELS];
    //simulator->getCommands(u_wheels);

    //double time;
    //simulator->getTime(time);

    /* Print simulation time every 5 sec */
    //if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    //{
    //    ROS_INFO("Simulator time: %d seconds", (int) time);
    //}

    /* Publish vehicle state */
    //std_msgs::Float64MultiArray vehicleStateMsg;
    //vehicleStateMsg.data.push_back(time);
    //vehicleStateMsg.data.push_back(x);
    //vehicleStateMsg.data.push_back(y);
    //vehicleStateMsg.data.push_back(theta);
    //vehicleStateMsg.data.push_back(velocity_act);
    //vehicleStateMsg.data.push_back(steer_act);
    //bar_publisher.publish(vehicleStateMsg);

    /* Publish clock */
    //rosgraph_msgs::Clock clockMsg;
    //clockMsg.clock = ros::Time(time);
    //clock_publisher.publish(clockMsg);
}

#include "omni_robot/omni_model.h"
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
    
    FullParamName = ros::this_node::getName()+"/N";
    if (false == Handle.getParam(FullParamName, N))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    WheelStates_sub = Handle.subscribe("/wheel_states", 1000, &omni_model::WheelStates_MessageCallback, this);
    RobotPose_sub = Handle.subscribe("/robot/pose", 1000, &omni_model::RobotPose_MessageCallback, this);
    CmdVel_pub = Handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
    Odom_pub = Handle.advertise<nav_msgs::Odometry>("/odom", 1000);
    WheelsRpm_pub = Handle.advertise<omni_robot::omni_msg>("wheels_bag_rpm", 1000);

    /* ROS services */
    Reset_srv = Handle.advertiseService("reset", &omni_model::reset_callback, this);

    /* Dynamic server */
    f = boost::bind(&omni_model::odom_callback, this, _1, _2);
    dynServer.setCallback(f);

    /* ROS Timer */
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
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void omni_model::RobotPose_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    /*ROS_INFO("[POSE] x: %.4f", msg->pose.position.x);
    ROS_INFO("[POSE] y: %.4f", msg->pose.position.y);
    ROS_INFO("[POSE] z: %.4f", msg->pose.position.z);*/
}

void omni_model::WheelStates_MessageCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    bool USE_TICKS = false;

    /* node time update */
    curr_time = msg->header.stamp;
    dt = (curr_time - prev_time).toSec();
    if (DEBUG) {
        ROS_INFO("[TIME] Previous time: %.4f", prev_time.toSec());
        ROS_INFO("[TIME] Current time: %.4f", curr_time.toSec());
        ROS_INFO("[TIME] Sampling Ts: %.4f", dt);
    }
    prev_time = curr_time;

    for (int i = 0; i < WHEELS; i++) {
        curr_tick[i] = msg->position.at(i);
        if (USE_TICKS) {
            // ticks to wheel angular velocity, radians/second (more accurate)
            // compute wheel angular velocity with number of ticks on motor encoder,
            // divided by total number of enc * time passed * gear ratio  //360 * 4096 * T;
            u_wheel[i] = (2*3.14159265*(curr_tick[i] - prev_tick[i])) / (dt * T * N);
        } else {
            // motor rotation (radians/minute), converted to the wheel and into rad/s - noisy
            u_wheel[i] = msg->velocity.at(i) / 60 / T; // extract motor(?) RPM, convert to rev per second, divide by gear ratio
        }
        prev_tick[i] = curr_tick[i];

        if (true)
            ROS_INFO("[WHEEL-%i] u: %.4f rpm", i + 1, u_wheel[i]);
    }

    wheels_rpm_msg.rpm_fl = u_wheel[0];
    wheels_rpm_msg.rpm_fr = u_wheel[1];
    wheels_rpm_msg.rpm_rr = u_wheel[2];
    wheels_rpm_msg.rpm_rl = u_wheel[3];
    WheelsRpm_pub.publish(wheels_rpm_msg);

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

    switch(odom_method) {

        case EULER: 
            // Euler
            ROS_INFO("[DYN-CONF] EULER ODOMETRY");
            x += delta_x * std::cos(theta) - delta_y * std::sin(theta);
            y += delta_x * std::sin(theta) + delta_y * std::cos(theta);
            break;

        case RUNGE_KUTTA:
            // Runge-Kutta
            ROS_INFO("[DYN-CONF] RUNGE-KUTTA ODOMETRY");
            x += delta_x * std::cos(theta + ang_vel * dt / 2) - delta_y * std::sin(theta + ang_vel * dt / 2);
            y += delta_x * std::sin(theta + ang_vel * dt / 2) + delta_y * std::cos(theta + ang_vel * dt / 2);
            break;
    }

    // heading update is the same for both integration methods
    theta += delta_theta;

    if (DEBUG) {
        ROS_INFO("[ODOM] x: %.4f", x);
        ROS_INFO("[ODOM] y: %.4f", y);
        ROS_INFO("[ODOM] theta: %.4f", theta);
    }

    // odometry position published into topic '/odom'
    odom_msg.header.stamp = curr_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

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
    transformStamped.header.frame_id = "odom";
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

bool omni_model::reset_callback(omni_robot::omni_reset::Request &req, omni_robot::omni_reset::Response &res) {
    res.x_old = this->x;
    res.y_old = this->y;
    res.theta_old = this->theta;

    this->x = req.x_new;
    this->y = req.y_new;
    this->theta = req.theta_new;

    ROS_INFO("Request to reset odometry to [x=%.4f, y=%.4f, theta=%.4f] - Responding with old odometry: [x=%.4f, y=%.4f, theta=%.4f]", req.x_new, req.y_new, req.theta_new, res.x_old, res.y_old, res.theta_old);

    return true;
}

void omni_model::odom_callback(omni_robot::parametersConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %s", config.odometry == 0 ? "Euler" : "Runge-Kutta");
    this->odom_method = config.odometry;
}

void omni_model::PeriodicTask(void)
{
    ;
}

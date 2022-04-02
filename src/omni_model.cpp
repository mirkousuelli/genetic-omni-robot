#include "omni-robot/omni_model.h"
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <math.h>
//#include <tf/transform_broadcaster.h>


void omni_model::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    /* loading parameter from the yalm server */
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

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
    prev = ros::Time::now();
    curr = ros::Time::now();

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
    for (int i = 0; i < WHEELS; i++) {
        ticks[i] = msg->position.at(i);
        ROS_INFO("[WHEEL-%i] tick: %.4f", i + 1, msg->position.at(i));
    }

    for (int i = 0; i < WHEELS; i++) {
        rpms[i] = msg->velocity.at(i);
        ROS_INFO("[WHEEL-%i] rpm: %.4f", i + 1, msg->velocity.at(i));
    }

    lin_vel_x = (r / 4) * (rpms[0] + rpms[1] + rpms[2] + rpms[3]);
    lin_vel_y = (r / 4) * (-rpms[0] + rpms[1] + rpms[2] + -rpms[3]);
    ang_vel = (r / 4) * (1 / (l + w)) * (-rpms[0] + rpms[1] - rpms[2] + rpms[3]);
    ROS_INFO("[FORWARD-KIN] linear velocity x: %.4f", lin_vel_x);
    ROS_INFO("[FORWARD-KIN] linear velocity y: %.4f", lin_vel_y);
    ROS_INFO("[FORWARD-KIN] angular velocity z: %.4f", ang_vel);

    lin_vel = sqrt(pow(lin_vel_x, 2) + pow(lin_vel_y, 2));
    cmd_vel_msg.twist.linear.x = lin_vel_x;
    cmd_vel_msg.twist.linear.y = lin_vel_y;
    cmd_vel_msg.twist.linear.z = 0.0;
    cmd_vel_msg.twist.angular.x = 0.0;
    cmd_vel_msg.twist.angular.y = 0.0;
    cmd_vel_msg.twist.angular.z = ang_vel;

    CmdVel_pub.publish(cmd_vel_msg);

    curr = ros::Time::now();
    dt = curr.toSec() - prev.toSec();
    ROS_INFO("[TIME] Previous time: %.4f", prev.toSec());
    ROS_INFO("[TIME] Current time: %.4f", curr.toSec());
    ROS_INFO("[TIME] Sampling Ts: %.4f", dt);
    prev = curr;

    if (true) {
        // Euler
        x += lin_vel * dt * std::cos(theta);
        y += lin_vel * dt * std::sin(theta);
        theta += ang_vel * dt;
        ROS_INFO("[ODOM-EULER] x: %.4f", x);
        ROS_INFO("[ODOM-EULER] y: %.4f", y);
        ROS_INFO("[ODOM-EULER] theta: %.4f", theta);
    } else {
        // Runge-Kutta
        x += lin_vel_x * dt * std::cos(theta + ang_vel * dt * 0.5);
        y += lin_vel_y * dt * std::sin(theta + ang_vel * dt * 0.5);
        theta += ang_vel * dt;
        ROS_INFO("[ODOM-KR] x: %.4f", x);
        ROS_INFO("[ODOM-KR] y: %.4f", y);
        ROS_INFO("[ODOM-KR] theta: %.4f", theta);
    }

    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";
    odom_msg.header.stamp = curr;  // ang_vel = cmd_vel_msg.twist.angular.z;
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

    // set header
    transformStamped.header.stamp = curr;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    // set x,y
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    // set theta
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    // send transform
    br.sendTransform(transformStamped);

    std::cout << std::endl;
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

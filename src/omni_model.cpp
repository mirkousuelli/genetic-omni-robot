#include "omni-robot/omni_model.h"
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <math.h>


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
    WheelStates_sub = Handle.subscribe("/wheel_states", 1, &omni_model::WheelStates_MessageCallback, this);
    RobotPose_sub = Handle.subscribe("/robot/pose", 1, &omni_model::RobotPose_MessageCallback, this);
    CmdVel_pub = Handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
    //clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    /* Create simulator class */
    simulator = new omni_odometry(dt);

    /* Initialize simulator class */
    simulator->setInitialState(x, y, theta);
    simulator->setOmniParams(r, l, w, T);

    lw = l + w;
    inv_lw = 1 / lw;

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

void omni_model::WheelStates_MessageCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < WHEELS; i++) {
        this->ticks[i] = msg->position.at(i);
    }

    for (int i = 0; i < WHEELS; i++) {
        this->rpms[i] = msg->velocity.at(i);
    }

    this->cmd_vel_msg.twist.linear.x = (r / 4) * (this->rpms[0] + this->rpms[1] + this->rpms[2] + this->rpms[3]);
    this->cmd_vel_msg.twist.linear.y = (r / 4) * (-this->rpms[0] + this->rpms[1] + this->rpms[2] - this->rpms[3]);
    this->cmd_vel_msg.twist.angular.z = (r / 4) * inv_lw * (-this->rpms[0] + this->rpms[1] - this->rpms[2] + this->rpms[3]);

    CmdVel_pub.publish(cmd_vel_msg);

    lin_vel = sqrt(pow(this->cmd_vel_msg.twist.linear.x, 2) + pow(cmd_vel_msg.twist.linear.y, 2));
    ang_vel = this->cmd_vel_msg.twist.angular.z;

    curr = ros::Time::now();

    dt = curr.toSec() - prev.toSec();

    // Euler
    x += lin_vel * dt * std::cos(theta);
    y += lin_vel * dt * std::sin(theta);
    theta += ang_vel * dt;

    prev = curr;

    /*ROS_INFO("[WHEEL-1] tick 1: %.2f", msg->position.at(0));
    ROS_INFO("[WHEEL-2] tick 2: %.2f", msg->position.at(1));
    ROS_INFO("[WHEEL-3] tick 3: %.2f", msg->position.at(2));
    ROS_INFO("[WHEEL-4] tick 4: %.2f", msg->position.at(3));
    
    ROS_INFO("[WHEEL-1] rpm 1: %.2f", msg->velocity.at(0));
    ROS_INFO("[WHEEL-2] rpm 2: %.2f", msg->velocity.at(1));
    ROS_INFO("[WHEEL-3] rpm 3: %.2f", msg->velocity.at(2));
    ROS_INFO("[WHEEL-4] rpm 4: %.2f", msg->velocity.at(3));*/
}

void omni_model::RobotPose_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //ROS_INFO("[POSE] \nPose: %s \n", msg->pose);
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

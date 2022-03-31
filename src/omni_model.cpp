#include "omni-robot/omni_model.h"

#include <unistd.h>


void omni_model::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    /* loading parameter from the yalm server */
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta0";
    if (false == Handle.getParam(FullParamName, theta0))
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
    foo_subscriber = Handle.subscribe("/foo", 1, &omni_model::foo_MessageCallback, this);
    bar_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/bar", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    /* Create simulator class */
    simulator = new omni_odometry(dt);

    /* Initialize simulator class */
    simulator->setInitialState(x0, y0, theta0);
    simulator->setOmniParams(r, l, w, T);

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

void omni_model::foo_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    /* Set vehicle commands */
    simulator->setReferenceCommands(msg->data.at(1), msg->data.at(2));
}

void omni_model::PeriodicTask(void)
{
    /* Integrate the model */
    simulator->integrate();

    /* Extract measurement from simulator */
    double x, y, theta;
    simulator->getPose(x, y, theta);

    double velocity_act, steer_act;
    simulator->getCommands(velocity_act, steer_act);

    double time;
    simulator->getTime(time);

    /* Print simulation time every 5 sec */
    if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    /* Publish vehicle state */
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(velocity_act);
    vehicleStateMsg.data.push_back(steer_act);
    bar_publisher.publish(vehicleStateMsg);

    /* Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}

#include "omni-robot/omni_tester.h"

void omni_tester::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    /* Initialize node state */
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    RunPeriod = dt;
    
    /* ROS topics */
    foo_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/foo", 1);
    bar_subscriber = Handle.subscribe("/bar", 1, &omni_tester::bar_MessageCallback, this);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
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

void omni_tester::bar_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // to be done...
}

void omni_tester::PeriodicTask(void)
{
    /* trajectory generation */
    // to be done

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer rate of change, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    //msg.data.push_back(speed);
    //msg.data.push_back(steer);
    foo_publisher.publish(msg);
}

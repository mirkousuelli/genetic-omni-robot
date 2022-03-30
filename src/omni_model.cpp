#include "car_traj_ctrl/car_traj_ctrl.h"

#include <unistd.h>


void car_traj_ctrl::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle parameters
    FullParamName = ros::this_node::getName()+"/L";
    if (false == Handle.getParam(FullParamName, L))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle initial state
    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta0";
    if (false == Handle.getParam(FullParamName, theta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleCommand_subscriber = Handle.subscribe("/car_input", 1, &car_traj_ctrl::vehicleCommand_MessageCallback, this);
    vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_state", 1);
    vehicleHeading_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_heading", 1);
    vehiclePosition_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_position", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    /* Create simulator class */
    simulator = new bicycle_ode(dt);

    /* Initialize simulator class */
    simulator->setInitialState(x0, y0, theta0);
    simulator->setVehicleParams(L);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void car_traj_ctrl::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        usleep(1000);
    }
}

void car_traj_ctrl::Shutdown(void)
{
    // Delete ode object
    delete simulator;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_traj_ctrl::vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    /*  Set vehicle commands */
    simulator->setReferenceCommands(msg->data.at(1), msg->data.at(2));
}

void car_traj_ctrl::PeriodicTask(void)
{
    /*  Integrate the model */
    simulator->integrate();

    /*  Extract measurement from simulator */
    double x, y, theta;
    simulator->getPose(x, y, theta);

    double velocity_act, steer_act;
    simulator->getCommands(velocity_act, steer_act);

    double time;
    simulator->getTime(time);

    /*  Print simulation time every 5 sec */
    if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    /*  Publish vehicle state */
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(velocity_act);
    vehicleStateMsg.data.push_back(steer_act);
    vehicleState_publisher.publish(vehicleStateMsg);

    /* Publishing vehicle heading */
    std_msgs::Float64MultiArray vehicleHeadingMsg;
    vehicleHeadingMsg.data.push_back(time);
    vehicleHeadingMsg.data.push_back(theta);
    vehicleHeading_publisher.publish(vehicleHeadingMsg);

    xp = x + eps * std::cos(theta);
    yp = y + eps * std::sin(theta);

    /* Publishing vehicle position of the chasis point as feedback */
    std_msgs::Float64MultiArray vehiclePositionMsg;
    vehiclePositionMsg.data.push_back(time);
    vehiclePositionMsg.data.push_back(xp);
    vehiclePositionMsg.data.push_back(yp);
    vehiclePosition_publisher.publish(vehiclePositionMsg);

    /*  Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}

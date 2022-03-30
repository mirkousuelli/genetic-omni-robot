#include "car_traj_ctrl/test_car_eight.h"

void test_car_eight::Prepare(void)
{
    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_input", 1);

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    /* Initialize node state */

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    RunPeriod = dt;

    // Vehicle parameters
    FullParamName = ros::this_node::getName()+"/L";
    if (false == Handle.getParam(FullParamName, L))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle initial state
    FullParamName = ros::this_node::getName()+"/a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta";
    if (false == Handle.getParam(FullParamName, theta))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/eps";
    if (false == Handle.getParam(FullParamName, eps))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpx";
    if (false == Handle.getParam(FullParamName, Kpx))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpy";
    if (false == Handle.getParam(FullParamName, Kpy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/Tix";
    if (false == Handle.getParam(FullParamName, Tix))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Tiy";
    if (false == Handle.getParam(FullParamName, Tiy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Ts";
    if (false == Handle.getParam(FullParamName, Ts))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleHeading_subscriber = Handle.subscribe("/car_heading", 1, &test_car_eight::vehicleHeading_MessageCallback, this);
    vehiclePosition_subscriber = Handle.subscribe("/car_position", 1, &test_car_eight::vehiclePosition_MessageCallback, this);

    w = 2 * PI / T;

    steer = speed = 0.0;
    xp = yp = 0.0;
    err_x = err_y = 0.0;
    I_x = I_y = 0.0;
    max_err_x = max_err_y = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_car_eight::RunPeriodically(float Period)
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

void test_car_eight::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_car_eight::vehicleHeading_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    /*  Set vehicle heading */
    theta = msg->data.at(1);
}

void test_car_eight::vehiclePosition_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    /*  Set vehicle position */
    xp = msg->data.at(1);
    yp = msg->data.at(2);
}

void test_car_eight::PeriodicTask(void)
{
    /* 8-shaped trajectory generation */

    // Trajectory computation
    xref    = a*std::sin(w*ros::Time::now().toSec());
    dxref   = w*a*std::cos(w*ros::Time::now().toSec());
    yref    = a*std::sin(w*ros::Time::now().toSec())*std::cos(w*ros::Time::now().toSec());
    dyref   = w*a*(std::pow(std::cos(w*ros::Time::now().toSec()),2.0)-std::pow(std::sin(w*ros::Time::now().toSec()),2.0));

    // position error
    err_x = (xref - xp);
    err_y = (yref - yp);

    if (max_err_x < std::fabs(err_x) || max_err_y < std::fabs(err_y)) {
        if (max_err_x < std::fabs(err_x)) {
            max_err_x = std::fabs(err_x); 
        } else {
            max_err_y = std::fabs(err_y);
        }
        ROS_INFO("Last maximum error (X, Y) : (%.4f, %.4f)", max_err_x, max_err_y);
    }

    // integral component with Backward Euler discretization
    I_x += Ts * Kpx * err_x / Tix;
    I_y += Ts * Kpy * err_y / Tiy;

    // Feedback linearization - PI Controller
    Vxp = dxref + Kpx * err_x + I_x;
    Vyp = dyref + Kpy * err_y + I_y;

    // Control law equations
    speed = Vxp * std::cos(theta) + Vyp * std::sin(theta);
    steer = std::atan((L * (Vyp * std::cos(theta) - Vxp * std::sin(theta))) / (eps * (Vxp * std::cos(theta) + Vyp * std::sin(theta))));

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer rate of change, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(speed);
    msg.data.push_back(steer);
    vehicleCommand_publisher.publish(msg);
}

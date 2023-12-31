#include <ros/ros.h>
#include "trajectory_navigation/nav_state_machine.hpp"

#include <utils/utils.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    NavigationStateMachine state_machine;

    ros::Rate rate(1);

    ROS_INFO("Launch Navigation State Machine");
    while (ros::ok())
    {
        state_machine.followPath();
        ros::spinOnce();
        rate.sleep();
    }

    // Example transitions
    state_machine.transitionToFollowPath();
    ROS_INFO("Current state: %d", state_machine.getCurrentState());

    state_machine.transitionToStop();
    ROS_INFO("Current state: %d", state_machine.getCurrentState());

    state_machine.transitionToIdle();
    ROS_INFO("Current state: %d", state_machine.getCurrentState());

    return 0;
}

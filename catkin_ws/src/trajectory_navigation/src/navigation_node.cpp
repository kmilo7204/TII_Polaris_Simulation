#include <ros/ros.h>
#include "trajectory_navigation/nav_state_machine.hpp"

#include <utils/utils.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    NavigationStateMachine state_machine;

    // Your ROS node logic goes here

    // Example transitions
    state_machine.transitionToFollowPath();
    ROS_INFO("Current state: %d", state_machine.getCurrentState());

    state_machine.transitionToStop();
    ROS_INFO("Current state: %d", state_machine.getCurrentState());

    state_machine.transitionToIdle();
    ROS_INFO("Current state: %d", state_machine.getCurrentState());

    printa();

    return 0;
}

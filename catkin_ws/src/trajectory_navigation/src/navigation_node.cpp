#include <ros/ros.h>
#include "trajectory_navigation/nav_state_machine.hpp"

#include <utils/utils.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    NavigationStateMachine state_machine;

    ros::Rate rate(1);

    ROS_INFO("Launch Navigation State Machine");
    while (ros::ok())
    {
        // if (path_.poses.size() > 0)
        // {
        //     follow_trajectory_1();
        // }
        // Start FSM
        // In the Start method I instante the object
        // Once instantiated, it passes to Idle state
        // If Idle state, checks the path size. If 0, continues in Idle
        // If path, transition Following state and call the follow path method
        // 
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

    printa();

    return 0;
}

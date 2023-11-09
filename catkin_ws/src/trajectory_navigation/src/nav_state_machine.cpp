#include "trajectory_navigation/nav_state_machine.hpp"

NavigationStateMachine::NavigationStateMachine() : current_state_(IDLE) {
    // Initialize trajectory followers
    // trajectory_followers_.push_back(std::make_shared<SimpleTrajectoryFollower>());
    // trajectory_followers_.push_back(std::make_shared<AdvancedTrajectoryFollower>());
}

void NavigationStateMachine::transitionToIdle() {
    ROS_INFO("Transitioning to IDLE state");
    current_state_ = IDLE;
    stopCurrentTrajectoryFollower();
}

void NavigationStateMachine::transitionToFollowPath() {
    ROS_INFO("Transitioning to FOLLOW_PATH state");
    current_state_ = FOLLOW_PATH;
    startCurrentTrajectoryFollower();
}

void NavigationStateMachine::transitionToStop() {
    ROS_INFO("Transitioning to STOP state");
    current_state_ = STOP;
    stopCurrentTrajectoryFollower();
}

NavigationState NavigationStateMachine::getCurrentState() const {
    return current_state_;
}

void NavigationStateMachine::startCurrentTrajectoryFollower() {
    // if (current_follower_index_ < trajectory_followers_.size()) {
    //     trajectory_followers_[current_follower_index_]->followPath();
    // }
}

void NavigationStateMachine::stopCurrentTrajectoryFollower() {
//     if (current_follower_index_ < trajectory_followers_.size()) {
//         trajectory_followers_[current_follower_index_]->stop();
//     }
}

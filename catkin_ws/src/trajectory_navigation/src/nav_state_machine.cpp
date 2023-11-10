#include "trajectory_navigation/nav_state_machine.hpp"

NavigationStateMachine::NavigationStateMachine() : current_state_(IDLE) {
    ROS_INFO("Transitioning to IDLE state");
    path_tracker_ = std::make_shared<PurePursuitTracker>();
    // Initialize trajectory followers
    // trajectory_followers_.push_back(std::make_shared<SimpleTrajectoryFollower>());
    // trajectory_followers_.push_back(std::make_shared<AdvancedTrajectoryFollower>());
}

void NavigationStateMachine::followPath()
{
    // Create the object
    // path_tracker_ = std::make_shared<PurePursuitTracker>();
    if (path_tracker_->hasPath())
    {
        transitionToFollowPath();
    }
    // else if (tracker.stopCondition())
    // {
    //     transitionToStop();
    // }
    else
    {
        transitionToIdle();
    }
}


void NavigationStateMachine::transitionToIdle()
{
    if (current_state_ != IDLE)
    {
        ROS_INFO("Transitioning to IDLE state");
        current_state_ = IDLE;
        stopCurrentTrajectoryFollower();
    }
}

void NavigationStateMachine::transitionToFollowPath()
{
    ROS_INFO("Transitioning to FOLLOW_PATH state");
    current_state_ = FOLLOW_PATH;
    startCurrentTrajectoryFollower();
}

void NavigationStateMachine::transitionToStop()
{
    ROS_INFO("Transitioning to STOP state");
    current_state_ = STOP;
    stopCurrentTrajectoryFollower();
}

NavigationState NavigationStateMachine::getCurrentState() const
{
    return current_state_;
}

void NavigationStateMachine::startCurrentTrajectoryFollower()
{
    path_tracker_->followPath();
    // if (current_follower_index_ < trajectory_followers_.size())
    // {
    //     trajectory_followers_[current_follower_index_]->followPath();
    // }
}

void NavigationStateMachine::stopCurrentTrajectoryFollower()
{
    path_tracker_->stop();
    // if (current_follower_index_ < trajectory_followers_.size())
    // {
    //     trajectory_followers_[current_follower_index_]->stop();
    // }
}

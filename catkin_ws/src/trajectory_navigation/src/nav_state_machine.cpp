#include "trajectory_navigation/nav_state_machine.hpp"

NavigationStateMachine::NavigationStateMachine() : current_state_(IDLE)
{
    ROS_INFO("Transitioning to IDLE state");
    // path_tracker_ = std::make_shared<PurePursuitTracker>();
    path_tracker_ = std::make_shared<PurePursuitTracker>();
    // Initialize trajectory followers
    // trajectory_followers_.push_back(std::make_shared<SimpleTrajectoryFollower>());
    // trajectory_followers_.push_back(std::make_shared<AdvancedTrajectoryFollower>());
}


void NavigationStateMachine::followPath()
{
    if (path_tracker_->hasPath()) // Si no tiene path deberia estar en idle
    {
        transitionToFollowPath();
    }
    else if (path_tracker_->stopCondition())
    {
        transitionToStop();
    }
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
    if (current_state_ != STOP)
    {
        ROS_INFO("Transitioning to STOP state");
        current_state_ = STOP;
        stopCurrentTrajectoryFollower();
    }
}


NavigationState NavigationStateMachine::getCurrentState() const
{
    return current_state_;
}


void NavigationStateMachine::startCurrentTrajectoryFollower()
{
    // This needs a fix
    path_tracker_->followPath();
}


void NavigationStateMachine::stopCurrentTrajectoryFollower()
{
    path_tracker_->stop();
}

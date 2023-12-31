#ifndef NAVIGATION_STATE_MACHINE_H
#define NAVIGATION_STATE_MACHINE_H

#include <ros/ros.h>
#include <memory>

#include <trajectory_navigation/pure_pursuit_tracker.hpp>
#include <trajectory_navigation/stanley_tracker.hpp>

enum NavigationState
{
    IDLE,
    FOLLOW_PATH,
    STOP
};

class NavigationStateMachine
{
public:
    NavigationStateMachine();

    void followPath();
    void transitionToIdle();
    void transitionToFollowPath();
    void transitionToStop();

    NavigationState getCurrentState() const;

private:
    NavigationState current_state_;

    // Create the object
    std::shared_ptr<PathTracker> path_tracker_;
    size_t current_follower_index_ = 0;

    void startCurrentTrajectoryFollower();
    void stopCurrentTrajectoryFollower();
};

#endif // NAVIGATION_STATE_MACHINE_H

// trajectory_follower_interface.h
#ifndef PATH_TRACKER_INTERFACE_H
#define PATH_TRACKER_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class PathTracker
{
public:
    virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) = 0;
    virtual void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) = 0;
    virtual void followPath() = 0;
    virtual void stop() = 0;
    virtual bool hasPath() = 0;

    virtual bool stopCondition() { return stop_conditon_; }
    virtual void setStopCondition(bool state) { stop_conditon_ = state; };

    virtual ~PathTracker() {}

    // Member variables
    nav_msgs::Odometry odom_;
    nav_msgs::Path path_;
    bool stop_conditon_ { false };
};

#endif  // PATH_TRACKER_INTERFACE_H

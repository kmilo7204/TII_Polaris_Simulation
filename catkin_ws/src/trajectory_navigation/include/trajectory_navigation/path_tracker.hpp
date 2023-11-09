// trajectory_follower_interface.h
#ifndef PATH_TRACKER_INTERFACE_H
#define PATH_TRACKER_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


class PathTracker
{
public:
    virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) = 0;
    virtual void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) = 0;
    // virtual void followPath(const geometry_msgs::PoseStamped& path) = 0;
    virtual void followPath() = 0;
    virtual void stop() = 0;
    virtual bool hasPath() = 0;

    virtual ~PathTracker() {}
};

#endif  // PATH_TRACKER_INTERFACE_H

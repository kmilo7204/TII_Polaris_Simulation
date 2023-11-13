#ifndef STANLEY_TRACKER_H
#define STANLEY_TRACKER_H

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "path_tracker.hpp"

class StanleyTracker : public PathTracker
{
public:
    StanleyTracker();
    void followPath() override;
    void stop() override;
    bool hasPath() override;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) override;
    void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) override;

    // Subscribers
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gps_subscriber_;

    // Publishers
    ros::Publisher ackermann_pub_;

    // Member variables
    const double wheelbase_{ 1.75 };

    nav_msgs::Odometry odom_;
    nav_msgs::Path path_;

    int prev_idx_ { 0 };
};

#endif

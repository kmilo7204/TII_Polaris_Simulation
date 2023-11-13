#ifndef PURE_PURSUIT_NODE_H
#define PURE_PURSUIT_NODE_H

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "path_tracker.hpp"

class PurePursuitTracker : public PathTracker
{
public:
    PurePursuitTracker();
    void followPath() override;
    void stop() override;
    bool hasPath() override;

private:
    // Subcribers callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) override;
    void pathCallback(const nav_msgs::Path::ConstPtr &path_msg) override;

    // Member methods
    int findGoalIndex(const std::vector<int> &index_vct);

    // Subscribers
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gps_subscriber_;

    // Publishers
    ros::Publisher ackermann_pub_;

    // Member variables
    nav_msgs::Odometry odom_;
    nav_msgs::Path path_;

    // Waypoints
    std::vector<double> path_points_x_;
    std::vector<double> path_points_y_;
    std::vector<double> path_points_yaw_;
    std::vector<double> dist_arr_;

    const double wheelbase_{1.75};
    const double k_{0.5};

    double look_ahead_dist_{6.0};
    int prev_idx_{0};

    // Pose variables
    double curr_x_{0.0};
    double curr_y_{0.0};
    double curr_yaw_{0.0};
};

#endif

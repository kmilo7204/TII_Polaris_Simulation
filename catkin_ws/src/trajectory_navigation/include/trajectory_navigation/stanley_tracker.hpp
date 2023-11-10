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
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) override;
    void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) override;

    void run();
    void process_1();

private:
    // Member methods
    void follow_trajectory();
    void follow_trajectory_1();

    double pi_2_pi(double angle);
    std::tuple<double, double, double> quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion);

    // Subscribers
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gps_subscriber_;

    // Publishers
    ros::Publisher ackermann_pub_;

    // Member variables
    const double wheelbase_{ 1.75 };
    const double Kp { 0.01 }; // Proportional control gain
    const double Kd { 0.3 }; // Derivative control gain
    const double Ks { 0.05 }; // Speed control gain

    nav_msgs::Odometry odom_;
    nav_msgs::Path path_;
    std::vector<geometry_msgs::PoseStamped> path_vct_;

    int prev_idx { 0 };


};

#endif
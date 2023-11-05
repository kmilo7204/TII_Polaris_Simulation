#ifndef PURE_PURSUIT_NODE_H
#define PURE_PURSUIT_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>

class PurePursuit {
public:
    PurePursuit();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& path_msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& gps_msg);
    void run();
    void process();

private:
    std::tuple<double, double, double> quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion);
    double find_angle(const std::vector<double>& v1, const std::vector<double>& v2);

private:
    // Subscribers
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gps_subscriber_;

    // Publishers
    ros::Publisher ackermann_pub_;

    // Member variables
    nav_msgs::Odometry odom_;
    nav_msgs::Path path_;

    std::vector<geometry_msgs::PoseStamped> path_vct_;

    // Waypoints
    std::vector<double> path_points_x_;
    std::vector<double> path_points_y_;
    std::vector<double> path_points_yaw_;
    std::vector<double> dist_arr_;

    // double look_ahead_dist_ {6.0};
    double look_ahead_dist_ {4.5};

};

#endif

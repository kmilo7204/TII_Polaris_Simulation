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

    void readWaypoints();

private:
    // Subscribers
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gps_subscriber_;

    // Publishers
    ros::Publisher ackermann_pub_;

    // Member variables
    nav_msgs::Odometry odom_;

    // Waypoints
    std::vector<double> path_points_x_;
    std::vector<double> path_points_y_;
    std::vector<double> path_points_yaw_;
    std::vector<double> dist_arr_;
};

#endif

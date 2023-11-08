#ifndef STANLEY_NODE_H
#define STANLEY_NODE_H

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


class Stanley
{
public:
    Stanley();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& path_msg);
    void run();
    void process();
    void process_1();

    double pi_2_pi(double angle);

std::tuple<double, double, double> quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion);

private:
    // Subscribers
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gps_subscriber_;

    // Publishers
    ros::Publisher ackermann_pub_;

    // Member variables
    nav_msgs::Odometry odom_;

    std::vector<geometry_msgs::PoseStamped> path_vct_;

    double wheelbase_{ 1.75 };

    // Control constants
    // const double Kp = 1.0; // Proportional control gain
    // const double Kd = 2.0; // Derivative control gain
    // const double Ks = 1.0; // Speed control gain

    const double Kp = 0.01; // Proportional control gain
    const double Kd = 0.3; // Derivative control gain
    const double Ks = 0.05; // Speed control gain

    int prev_idx {0};

};

#endif

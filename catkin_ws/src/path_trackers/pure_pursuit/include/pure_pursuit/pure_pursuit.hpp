#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
// #include <ackerman_msgs/AckermanDrive.h>
#include <ackermann_msgs/AckermannDrive.h>

class PurePursuit {
public:
    PurePursuit();
    void odomCallback(const nav_msgs::Path::ConstPtr& path_msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
    void run();
private:
    // Subscribers
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gps_subscriber_;

    // Publishers
    ros::Publisher ackermann_pub_;
};

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>

class PurePursuit {
public:
    PurePursuit();
    void pathCallback(const nav_msgs::Path::ConstPtr& path_msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
    void run();
private:
    ros::Subscriber path_subscriber_;
    ros::Subscriber gps_subscriber_;
    ros::Publisher control_publisher_;
};

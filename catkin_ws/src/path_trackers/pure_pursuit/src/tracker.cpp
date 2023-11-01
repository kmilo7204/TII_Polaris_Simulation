#include <pure_pursuit/tracker.hpp>

PurePursuit::PurePursuit()
{
  ;
}

void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ;
}

void PurePursuit::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
  ;
}

void PurePursuit::run()
{
  ;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit path_tracker;

    // Start the path-tracking node
    path_tracker.run();

    return 0;
}
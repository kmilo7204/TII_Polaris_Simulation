#include <ros/ros.h>
#include "path_generator/path_generator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_generator_node");
    PathGenerator path_generator;
    ros::Rate rate(1);

    ROS_INFO("Launch Path Generator");
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

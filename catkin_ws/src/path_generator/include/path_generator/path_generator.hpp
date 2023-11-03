#ifndef PATH_GENERATOR_NODE_H
#define PATH_GENERATOR_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


class PathGenerator
{
public:
  PathGenerator();
  void readWaypoints();

private:
    ros::Publisher path_pub_;
};

#endif

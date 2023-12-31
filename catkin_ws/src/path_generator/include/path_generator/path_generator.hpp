#ifndef PATH_GENERATOR_NODE_H
#define PATH_GENERATOR_NODE_H

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>


class PathGenerator
{
public:
  PathGenerator();
  nav_msgs::Path getPath();
  void publishPath();
  bool readAndPublishPathSvcCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

private:
  bool readWaypointsFromCSV(std::string csv_filename);

  ros::Publisher path_pub_;
  ros::ServiceServer path_req_srv_;

  nav_msgs::Path path_;
};

#endif

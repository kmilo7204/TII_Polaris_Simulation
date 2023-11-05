#include <path_generator/path_generator.hpp>

#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>


PathGenerator:: PathGenerator()
{
  // I can create a service server to handle request and publish the path
  ros::NodeHandle nh;

  // Publisher
  path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1);
}

nav_msgs::Path PathGenerator::readWaypoints()
{
  std::string filename = ros::package::getPath("path_generator") + "/files/wps.csv";
  std::ifstream file(filename);

  nav_msgs::Path path_msg;

  if (!file.is_open())
  {
    std::cerr << "Error: Unable to open waypoints file " << filename << std::endl;
    return path_msg;
  }

  std::vector<std::tuple<double, double, double>> path_points;
  std::string line;

  while (std::getline(file, line))
  {
    std::istringstream ss(line);

    double x;
    double y;
    double yaw;
    char comma;

    if(ss >> x >> comma >> y >> comma >> yaw)
    {
      geometry_msgs::PoseStamped waypoint;
      waypoint.pose.position.x = x;
      waypoint.pose.position.y = y;
      waypoint.pose.position.z = yaw;
      path_msg.poses.push_back(waypoint);
    }
    else
    {
      std::cout << "Extraction failed!" << std::endl;
    }
  }
  return path_msg;
}


void PathGenerator::publish()
{
  nav_msgs::Path path_msg = readWaypoints();

  path_msg.header.frame_id = "map";
  path_msg.header.stamp = ros::Time::now();

  ROS_INFO("Publishing path into /path topic");
  path_pub_.publish(path_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_generator_node");
    PathGenerator path_generator;
    ros::Rate rate(1);

    while (ros::ok())
    {
      path_generator.publish();
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
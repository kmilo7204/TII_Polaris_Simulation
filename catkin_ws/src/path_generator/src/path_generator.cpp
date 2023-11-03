#include <path_generator/path_generator.hpp>

#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>


PathGenerator:: PathGenerator()
{
  ros::NodeHandle nh;

  path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1);
}

void PathGenerator::readWaypoints()
{
  std::string filename = ros::package::getPath("path_tracker") + "/files/wps.csv";
  ROS_INFO("Filename: %s", filename.c_str());
  std::ifstream file(filename);

  if (!file.is_open())
  {
    std::cerr << "Error: Unable to open waypoints file " << filename << std::endl;
    return;
  }

  std::vector<std::tuple<double, double, double>> path_points;
  std::string line;

  while (std::getline(file, line))
  {
    std::istringstream ss(line);
    double x, y, yaw;
    if (ss >> x >> y >> yaw)
    {
      path_points.emplace_back(x, y, yaw);
    }
  }

  // Message that will carry the Path data
  nav_msgs::Path path_msg;

  for (const auto& point : path_points)
  {
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose.position.x = std::get<0>(point);
    waypoint.pose.position.y = std::get<1>(point);
    waypoint.pose.position.z = std::get<2>(point);
    path_msg.poses.push_back(waypoint);
  }

  // This might change
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = ros::Time::now();

  path_pub_.publish(path_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_generator_node");
    PathGenerator path_generator;

    // Start the path-tracking node
    path_generator.readWaypoints();

    return 0;
}
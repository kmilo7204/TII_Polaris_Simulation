#include <path_generator/path_generator.hpp>

#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>


#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PathGenerator:: PathGenerator()
{
  // I can create a service server to handle request and publish the path
  ros::NodeHandle nh;

  // Publisher
  path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1);
}


nav_msgs::Path PathGenerator::readWaypoints()
{
  std::string filename = ros::package::getPath("path_generator") + "/files/others.csv";
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
      // waypoint.pose.position.z = yaw;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      waypoint.pose.orientation.x = q.x();
      waypoint.pose.orientation.y = q.y();
      waypoint.pose.orientation.z = q.z();
      waypoint.pose.orientation.w = q.w();

      path_msg.poses.push_back(waypoint);
      ROS_INFO("X: %f, Y: %f, Z: %f, W: %f", q.x(), q.y(), q.z(), q.w());
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
  if (pub_ctr < 2)
  {
    nav_msgs::Path path_msg = readWaypoints();

    path_msg.header.frame_id = "base_footprint";
    path_msg.header.stamp = ros::Time::now();

    ROS_INFO("Publishing path into /path topic");
    path_pub_.publish(path_msg);
    pub_ctr++;
  }
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

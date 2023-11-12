#include <fstream>
#include <iostream>
#include <sstream>

#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <path_generator/path_generator.hpp>

PathGenerator::PathGenerator()
{
  // Create the nodehandler
  ros::NodeHandle nh;

  // Publisher
  path_pub_ = nh.advertise<nav_msgs::Path>("/path", 1);

  // Service server
  path_req_srv_ = nh.advertiseService("/generate_path", &PathGenerator::readAndPublishPathSvcCallback, this);
}


bool PathGenerator::readAndPublishPathSvcCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Path creation request received");
  if (readWaypointsFromCSV("wps.csv"))
  {
    // Publish the path
    publishPath();
    res.success = true;
    return true;
  }
  res.success = false;
  return true;
}


bool PathGenerator::readWaypointsFromCSV(std::string csv_filename)
{
  // Look at CSV files under the files folder
  std::string filename = ros::package::getPath("path_generator") + "/files/" + csv_filename;
  ROS_INFO("CSV path file found at: %s", filename);

  std::ifstream file(filename);

  if (!file.is_open())
  {
    std::cerr << "Error: Unable to open waypoints file " << filename << std::endl;
    return false;
  }

  ROS_INFO("Creating path message...");
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

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      waypoint.pose.orientation.x = q.x();
      waypoint.pose.orientation.y = q.y();
      waypoint.pose.orientation.z = q.z();
      waypoint.pose.orientation.w = q.w();

      path_.poses.push_back(waypoint);
      ROS_INFO("X: %f, Y: %f, Z: %f, W: %f", q.x(), q.y(), q.z(), q.w());
    }
    else
    {
      std::cout << "Extraction failed!" << std::endl;
    }
  }
  return true;
}


void PathGenerator::publishPath()
{
    path_.header.frame_id = "base_footprint";
    path_.header.stamp = ros::Time::now();

    ROS_INFO("Publishing path into /path topic");
    path_pub_.publish(path_);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_generator_node");
  PathGenerator path_generator;
  ros::Rate rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

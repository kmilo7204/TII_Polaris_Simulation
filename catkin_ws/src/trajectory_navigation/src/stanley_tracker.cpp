#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <trajectory_navigation/stanley_tracker.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <utils/utils.hpp>

StanleyTracker::StanleyTracker()
{
  ros::NodeHandle nh;

  odom_subscriber_ = nh.subscribe("/gem/base_footprint/odom", 1, &StanleyTracker::odomCallback, this);
  gps_subscriber_ = nh.subscribe("/path", 1, &StanleyTracker::pathCallback, this);

  ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
}

void StanleyTracker::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  odom_ = *odom_msg;
}

void StanleyTracker::pathCallback(const nav_msgs::Path::ConstPtr &path_msg)
{
  ROS_INFO("Receiving information in path message");
  path_ = *path_msg;
}

void StanleyTracker::stop()
{
  setStopCondition(false);

  // Send stop command
  ackermann_msgs::AckermannDrive ackermann_cmd;
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;
  ackermann_pub_.publish(ackermann_cmd);
}

bool StanleyTracker::hasPath()
{
  if (prev_idx_ >= (path_.poses.size() - 10) || (path_.poses.size() < 2))
  {
    return false;
  }
  return true;
}

void StanleyTracker::followPath()
{
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles(odom_.pose.pose.orientation);

  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  double curr_vx = odom_.twist.twist.linear.x;
  double curr_vy = odom_.twist.twist.linear.y;

  double front_x = wheelbase_ * std::cos(curr_yaw) + curr_x;
  double front_y = wheelbase_ * std::sin(curr_yaw) + curr_y;

  // ROS_INFO("front_x: %f, front_y: %f", front_x, front_y);

  std::vector<double> dist_x;
  std::vector<double> dist_y;
  dist_x.reserve(path_.poses.size());
  dist_y.reserve(path_.poses.size());

  std::vector<std::pair<int, double>> dist_vct;
  dist_vct.reserve(path_.poses.size());
  int i = 0;

  for (geometry_msgs::PoseStamped pose_stp : path_.poses)
  {
    double x = pose_stp.pose.position.x;
    double y = pose_stp.pose.position.y;

    dist_x.push_back(front_x - x);
    dist_y.push_back(front_y - y);

    double dist = sqrt(pow(front_x - x, 2) + pow(front_y - y, 2));

    dist_vct.push_back(std::make_pair(i, dist));
    i++;
  }

  auto min_pair = std::min_element(dist_vct.begin(), dist_vct.end(),
                                   [](const auto &a, const auto &b)
                                   {
                                     return a.second < b.second;
                                   });

  int index = min_pair->first;

  ackermann_msgs::AckermannDrive ackermann_cmd;
  // Publish control commands
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;

  if (index != (path_.poses.size() - 1) && index >= prev_idx_)
  {
    prev_idx_ = index;

    double front_axle_ort_x = std::cos(curr_yaw - M_PI_2);
    double front_axle_ort_y = std::sin(curr_yaw - M_PI_2);

    double target_path_x = dist_x[index];
    double target_path_y = dist_y[index];

    double cte = target_path_x * front_axle_ort_x + target_path_y * front_axle_ort_y;

    double path_x = path_.poses[index].pose.position.x;
    double path_y = path_.poses[index].pose.position.y;
    double path_x_next = path_.poses[index + 1].pose.position.x;
    double path_y_next = path_.poses[index + 1].pose.position.y;

    double theta_p = std::atan2(path_y_next - path_y, path_x_next - path_x);

    double f_vel = sqrt(pow(curr_vx, 2) + pow(curr_vy, 2));
    double theta_e = piToPi(theta_p - curr_yaw);
    double delta = theta_e + std::atan2(0.1 * cte, 1.2 * f_vel);

    // Enable for debugging
    // ROS_INFO("Theta path: %f", theta_p);
    // ROS_INFO("Forward velocity: %f", f_vel);
    // ROS_INFO("Theta error: %f", theta_e);
    // ROS_INFO("Delta: %f", delta);

    ROS_INFO("[curr_x: %f, curr_y: %f]", curr_x, curr_y);
    ROS_INFO("[goal_x: %f, goal_y: %f]", path_.poses[index].pose.position.x, path_.poses[index].pose.position.y);
    ROS_INFO("CTE: %f", cte);

    // Publish control commands
    ackermann_cmd.speed = 1.5;
    ackermann_cmd.steering_angle = delta;
  }
  else
  {
    setStopCondition(true);
  }
  ackermann_pub_.publish(ackermann_cmd);
}

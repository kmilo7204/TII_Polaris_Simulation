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

StanleyTracker::StanleyTracker()
{
  ros::NodeHandle nh;

  odom_subscriber_ = nh.subscribe("/gem/base_footprint/odom", 1, &StanleyTracker::odomCallback, this);
  gps_subscriber_ = nh.subscribe("/path", 1, &StanleyTracker::pathCallback, this);

  ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
}

void StanleyTracker::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  odom_ = *odom_msg;
}

void StanleyTracker::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ROS_INFO("Receiving information in path message");
  path_vct_ = path_msg->poses;
  path_ = *path_msg;
}

void StanleyTracker::run()
{
  ros::Rate rate(1);

  ROS_INFO("In run()");
  while (ros::ok())
  {
    if (path_.poses.size() > 0)
    {
      follow_trajectory_1();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

std::tuple<double, double, double> StanleyTracker::quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion)
{
    double qx = quaternion.x;
    double qy = quaternion.y;
    double qz = quaternion.z;
    double qw = quaternion.w;

    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    return std::make_tuple(roll, pitch, yaw);
}

double StanleyTracker::pi_2_pi(double angle)
{
  if (angle > M_PI)
  {
    return angle - 2.0 * M_PI;
  }
  if (angle < -M_PI)
  {
    return angle + 2.0 * M_PI;
  }

  return angle;
}

void StanleyTracker::process_1()
{
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles(odom_.pose.pose.orientation);

  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  // Find the nearest point on the path to the vehicle's position
  double min_dist = std::numeric_limits<double>::max();
  int target_index = 0;

  for (int i = 0; i < path_.poses.size(); i++)
  {
    double dx = (wheelbase_ * std::cos(curr_yaw) + curr_x) - path_.poses[i].pose.position.x;
    double dy = (wheelbase_ * std::sin(curr_yaw) + curr_y) - path_.poses[i].pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist)
    {
        min_dist = dist;
        target_index = i;
    }
  }

  ackermann_msgs::AckermannDrive ackermann_cmd;
  // Publish control commands
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;

  ROS_INFO("Index: %ld", target_index);

  if (target_index < (path_.poses.size() - 1) &&  target_index >= prev_idx)
  {
    prev_idx = target_index;
    // Calculate the cross-track error (cte)
    double cte = min_dist * sin(curr_yaw - atan2(curr_y - path_.poses[target_index].pose.position.y,
                                                  curr_x - path_.poses[target_index].pose.position.x));

    ROS_INFO("CTE: %f", cte);

    // Calculate the desired heading (reference angle)
    double ref_yaw = atan2(path_.poses[target_index + 1].pose.position.y - path_.poses[target_index].pose.position.y,
                          path_.poses[target_index + 1].pose.position.x - path_.poses[target_index].pose.position.x);

    double delta_yaw = curr_yaw - ref_yaw;

    // Ensure the angle difference is within the range [-pi, pi]
    while (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;

    // Calculate the steering angle
    double steer_angle = ref_yaw + atan2(0.3 * cte, 1.1 * odom_.twist.twist.linear.x + 0.6 * delta_yaw); //0.8
    // ROS_INFO("Velocity: %f", odom_.twist.twist.linear.x);
    ROS_INFO("Steer: %f", steer_angle);

    // Publish control commands
    ackermann_cmd.speed = 1.5;
    ackermann_cmd.steering_angle = steer_angle;
  }

  ackermann_pub_.publish(ackermann_cmd);
}

void StanleyTracker::follow_trajectory_1()
{
  ROS_INFO("PROCESS");
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles(odom_.pose.pose.orientation);

  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  ROS_INFO("curr_x: %f, curr_y: %f, curr_yaw: %f", curr_x, curr_y, curr_yaw);

  double curr_vx = odom_.twist.twist.linear.x;
  double curr_vy = odom_.twist.twist.linear.y;

  double front_x = wheelbase_ * std::cos(curr_yaw) + curr_x;
  double front_y = wheelbase_ * std::sin(curr_yaw) + curr_y;

  ROS_INFO("front_x: %f, front_y: %f", front_x, front_y);

  std::vector<double> dist_x;
  std::vector<double> dist_y;
  dist_x.reserve(path_vct_.size());
  dist_y.reserve(path_vct_.size());

  std::vector<std::pair<int, double>> dist_vct;
  dist_vct.reserve(path_vct_.size());
  int i = 0;

  for (geometry_msgs::PoseStamped pose_stp : path_vct_)
  {
    double x = pose_stp.pose.position.x;
    double y = pose_stp.pose.position.y;

    dist_x.push_back(front_x - x);
    dist_y.push_back(front_y - y);

    double dist = sqrt(pow(front_x - x, 2) + pow(front_y - y, 2));

    // ROS_INFO("Distance: %f", dist);

    dist_vct.push_back(std::make_pair(i, dist));
    i++;
  }

  // std::pair<int, double>::Iterator
  auto min_pair = std::min_element(dist_vct.begin(), dist_vct.end(),
    [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

  int index = min_pair->first;

  ROS_INFO("Index: %ld", index);

  ackermann_msgs::AckermannDrive ackermann_cmd;
  // Publish control commands
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;

  if (index != (path_vct_.size() - 1) &&  index >= prev_idx)
  {
    prev_idx = index;

    double front_axle_ort_x = std::cos(curr_yaw - M_PI_2);
    double front_axle_ort_y = std::sin(curr_yaw - M_PI_2);

    double target_path_x = dist_x[index];
    double target_path_y = dist_y[index];

    double cte = target_path_x * front_axle_ort_x + target_path_y * front_axle_ort_y;

    double path_x = path_vct_[index].pose.position.x;
    double path_y = path_vct_[index].pose.position.y;
    double path_x_next = path_vct_[index + 1].pose.position.x;
    double path_y_next = path_vct_[index + 1].pose.position.y;

    // Path angle
    // ROS_INFO("X_diff: %f, Y_diff: %f", path_x_next - path_x, path_y_next - path_y);

    double theta_p = std::atan2(path_y_next - path_y, path_x_next - path_x);
    ROS_INFO("Theta path: %f", theta_p);

    double f_vel = sqrt(pow(curr_vx, 2) + pow(curr_vy, 2));
    ROS_INFO("Forward velocity: %f", f_vel);

    double theta_e = pi_2_pi(theta_p - curr_yaw);
    ROS_INFO("Theta error: %f", theta_e);

    double delta = theta_e + std::atan2(0.1 * cte, 1.2 * f_vel);
    ROS_INFO("Delta: %f", delta);

    double theta_e_deg = theta_e * (180.0 / M_PI);
    ROS_INFO("CTE: %f, Heading error: %f", cte, theta_e_deg);

    // Publish control commands
    ROS_INFO("Setting speed");
    ackermann_cmd.speed = 1.5;
    ackermann_cmd.steering_angle = delta;
  }
  ackermann_pub_.publish(ackermann_cmd);
}


void StanleyTracker::follow_trajectory()
{
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles(odom_.pose.pose.orientation);

  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  // Find the nearest point on the path to the vehicle's position
  double min_dist = std::numeric_limits<double>::max();
  int target_index = 0;

  for (int i = 0; i < path_.poses.size(); i++)
  {
    double dx = (wheelbase_ * std::cos(curr_yaw) + curr_x) - path_.poses[i].pose.position.x;
    double dy = (wheelbase_ * std::sin(curr_yaw) + curr_y) - path_.poses[i].pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist)
    {
        min_dist = dist;
        target_index = i;
    }
  }

  ROS_INFO("Index: %ld", target_index);

  ackermann_msgs::AckermannDrive ackermann_cmd;
  // Publish control commands
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;

  if (target_index < (path_.poses.size() - 2) &&  target_index >= prev_idx)
  {
    prev_idx = target_index;

    // Orthogonal line to the path
    double front_axle_ort_x = std::cos(curr_yaw - M_PI_2);
    double front_axle_ort_y = std::sin(curr_yaw - M_PI_2);

    // Target path points
    double path_x = path_.poses[target_index].pose.position.x;
    double path_y = path_.poses[target_index].pose.position.y;
    double path_x_next = path_.poses[target_index + 1].pose.position.x;
    double path_y_next = path_.poses[target_index + 1].pose.position.y;

    // Distance to target = Front axle point - Path point
    double dist_target_x = ((wheelbase_ * std::cos(curr_yaw)) + curr_x) - path_x;
    double dist_target_y = ((wheelbase_ * std::sin(curr_yaw)) + curr_y) - path_y;

    // Cross tack error
    double cte = (dist_target_x * front_axle_ort_x) + (dist_target_y * front_axle_ort_y);

    // Path angle
    double theta_p = std::atan2(path_y_next - path_y, path_x_next - path_x);
    ROS_INFO("Theta path: %f", theta_p);

    // Forward velocity
    double f_vel = sqrt(pow(odom_.twist.twist.linear.x, 2) + pow(odom_.twist.twist.linear.y, 2));
    ROS_INFO("Forward velocity: %f", f_vel);

    // Error angle
    double theta_e = pi_2_pi(theta_p - curr_yaw);
    ROS_INFO("Theta error: %f", theta_e);

    // Steering command
    double delta = theta_e + std::atan2(0.1 * cte, 1.2 * f_vel);
    ROS_INFO("Delta: %f, CTE: %f", delta, cte);

    // double theta_e_deg = theta_e * (180.0 / M_PI);
    // ROS_INFO("CTE: %f, Heading error: %f", cte, theta_e_deg);

    // Publish control commands
    ackermann_cmd.speed = 1.5;
    ackermann_cmd.steering_angle = delta;
  }
  ackermann_pub_.publish(ackermann_cmd);
}

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <trajectory_navigation/pure_pursuit_tracker.hpp>


PurePursuitTracker::PurePursuitTracker()
{
  ros::NodeHandle nh;

  odom_subscriber_ = nh.subscribe("/gem/base_footprint/odom", 1, &PurePursuitTracker::odomCallback, this);
  gps_subscriber_ = nh.subscribe("/path", 1, &PurePursuitTracker::pathCallback, this);

  ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
}


std::tuple<double, double, double> PurePursuitTracker::quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion)
{
    double qx = quaternion.x;
    double qy = quaternion.y;
    double qz = quaternion.z;
    double qw = quaternion.w;

    // ROS_INFO("QX: %f, QY: %f, QZ: %f, QW: %f", qx, qy, qz, qw);

    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
    // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

    return std::make_tuple(roll, pitch, yaw);
}


void PurePursuitTracker::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  odom_ = *odom_msg;
}


void PurePursuitTracker::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ROS_INFO("Receiving information in path message");
  path_ = *path_msg;
  path_vct_ = path_msg->poses;
}


double PurePursuitTracker::find_angle(const std::vector<double>& v1, const std::vector<double>& v2)
{
  // Calculate the angle between two vectors v1 and v2
  double dot_product = v1[0] * v2[0] + v1[1] * v2[1];
  double magnitude_v1 = std::sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
  double magnitude_v2 = std::sqrt(v2[0] * v2[0] + v2[1] * v2[1]);
  return std::acos(dot_product / (magnitude_v1 * magnitude_v2));
}


void PurePursuitTracker::followPath()
{
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles(odom_.pose.pose.orientation);

  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  std::vector<double> dist_vct;
  // ROS_INFO("Path size: %d", path_vct_.size());
  dist_vct.reserve(path_vct_.size());

  // ROS_INFO("Dist size: %d", dist_vct.size());
  // Since we dont know what is the lower distance, we need to find it out.
  int idx = 0;
  for (const geometry_msgs::PoseStamped& pose_stamped : path_vct_)
  {
    double dist = sqrt(pow(pose_stamped.pose.position.x - curr_x, 2) + pow(pose_stamped.pose.position.y - curr_y, 2));
    // ROS_INFO("Distance: %f", dist);
    dist_vct.push_back(dist);
    idx++;
  }

  std::vector<int> goal_vct;
  for (int i = 0; i < dist_vct.size(); i++)
  {
    if (dist_vct[i] >= look_ahead_dist_ - 2.0 && dist_vct[i] <= look_ahead_dist_ + 2.0)
    {
      ROS_INFO("Index to be processed: %ld", i);
      goal_vct.push_back(i);
    }
  }

  ROS_INFO("Goals processed: %d", goal_vct.size());

  int goal = -1;
  for (int idx : goal_vct)
  {
    geometry_msgs::PoseStamped goal_pose = path_vct_[idx];
    double x_goal = goal_pose.pose.position.x;
    double y_goal = goal_pose.pose.position.y;

    std::vector<double> v1 = {x_goal - curr_x, y_goal - curr_y};
    std::vector<double> v2 = {std::cos(curr_yaw), std::sin(curr_yaw)};
    ROS_INFO("V1_x: %f, V1_y: %f", x_goal - curr_x, y_goal - curr_y);

    double temp_angle = find_angle(v1, v2);
    ROS_INFO("Temp angle: %f",temp_angle);

    if (std::abs(temp_angle) < ((M_PI / 2) + 0.5))
    {
      ROS_INFO("Previous index: %d", prev_idx_);
      if (prev_idx_ == 0)
      {
        goal = idx;
        prev_idx_ = idx;
      }

      if (idx >= prev_idx_)
      {
        goal = idx;
        prev_idx_ = idx;
        break;
      }
    }
  }

  ackermann_msgs::AckermannDrive ackermann_cmd;
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;

  if (goal != -1)
  {
    ROS_INFO("Goal index: %x", goal);

    double l = dist_vct[goal];
    geometry_msgs::PoseStamped target_pose = path_vct_[goal];
    ROS_INFO("curr_x: %f, curr_y: %f, curr_yaw: %f", curr_x, curr_y, curr_yaw);

    // Find the curvature
    std::tuple<double, double, double> goal_angles = quaternionToEulerAngles(target_pose.pose.orientation);

    double goal_yaw = std::get<2>(goal_angles);
    // ROS_INFO("Goal yaw: %f", goal_yaw);
    ROS_INFO("goal_x: %f, goal_y: %f, goal_yaw: %f", target_pose.pose.position.x, target_pose.pose.position.y, goal_yaw);

    double alpha = goal_yaw - curr_yaw;
    ROS_INFO("Alpha: %f", alpha);

    double angle_i = std::atan((2 * k_ * wheelbase_ * std::sin(alpha)) / l);

    ROS_INFO("Angle i: %f", angle_i);

    double angle = angle_i * 2;

    double ct_error = std::sin(alpha) * l;

    ROS_INFO("CTE: %f", ct_error);

    // Publish control commands
    ackermann_cmd.speed = 2.8;
    ackermann_cmd.steering_angle = angle;
  }
  // Set control_command values as needed
  ackermann_pub_.publish(ackermann_cmd);
}

void PurePursuitTracker::stop()
{
  path_vct_.clear();

  // Send stop command
  ackermann_msgs::AckermannDrive ackermann_cmd;
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;
  ackermann_pub_.publish(ackermann_cmd);
}

bool PurePursuitTracker::hasPath()
{
  return path_vct_.size() > 2;
}

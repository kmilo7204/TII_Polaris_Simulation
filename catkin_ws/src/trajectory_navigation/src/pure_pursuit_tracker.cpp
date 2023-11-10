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

#include <utils/utils.hpp>

PurePursuitTracker::PurePursuitTracker()
{
  ros::NodeHandle nh;

  odom_subscriber_ = nh.subscribe("/gem/base_footprint/odom", 1, &PurePursuitTracker::odomCallback, this);
  gps_subscriber_ = nh.subscribe("/path", 1, &PurePursuitTracker::pathCallback, this);

  ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
}


void PurePursuitTracker::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  odom_ = *odom_msg;
}


void PurePursuitTracker::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ROS_INFO("Receiving information in path message");
  path_ = *path_msg;
  // path_vct_ = path_msg->poses;
}


int PurePursuitTracker::findGoalIndex(const std::vector<int>& index_vct)
{
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles1(odom_.pose.pose.orientation);

  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  int goal = -1;
  for (int idx : index_vct)
  {
    geometry_msgs::PoseStamped goal_pose = path_.poses[idx];
    double x_goal = goal_pose.pose.position.x;
    double y_goal = goal_pose.pose.position.y;

    // Path vector
    std::vector<double> vector_1 = { x_goal - curr_x, y_goal - curr_y };
    // Heading vector
    std::vector<double> vector_2 = { std::cos(curr_yaw), std::sin(curr_yaw) };
    ROS_INFO("V1_x: %f, V1_y: %f", x_goal - curr_x, y_goal - curr_y);

    double temp_angle = angleBetweenVectors(vector_1, vector_2);
    ROS_INFO("Temp angle: %f",temp_angle);

    if (std::abs(temp_angle) < ((M_PI / 2) + 0.5))
    {
      ROS_INFO("Previous index: %d", prev_idx_);
      // First assignation when there is a waypoint in the look ahead distance
      if (prev_idx_ == 0)
      {
        goal = idx;
        prev_idx_ = idx;
      }
      // We only want to move forward, so dismiss already processed waypoints
      if (idx >= prev_idx_)
      {
        goal = idx;
        prev_idx_ = idx;
        break;
      }
    }
  }
  return goal;
}


void PurePursuitTracker::followPath()
{
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles1(odom_.pose.pose.orientation);

  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  std::vector<double> dist_vct;
  dist_vct.reserve(path_.poses.size());

  // ROS_INFO("Dist size: %d", dist_vct.size());
  int idx = 0;
  std::vector<int> goal_vct;
  // I can also start processing this from the previous index instead of checking again all the things
  // for (const geometry_msgs::PoseStamped& pose_stamped : path_.poses)
  // int start_idx = prev_idx_ ?
  // I can set a bottom index to solve this, based on the ones that can be processed
  for (int i = 0; i < path_.poses.size(); ++i)
  {
    double dx = path_.poses[i].pose.position.x - curr_x;
    double dy = path_.poses[i].pose.position.y - curr_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist >= look_ahead_dist_ - 2.0 && dist <= look_ahead_dist_ + 2.0)
    {
      ROS_INFO("Can be processed: %ld", i);
      goal_vct.push_back(i);
    }
    // ROS_INFO("Distance: %f", dist);
    dist_vct.push_back(dist);
    idx++;
  }

  ROS_INFO("Goals processed: %d", goal_vct.size());

  // Find the goal index based in distance and path angle
  int goal_index = findGoalIndex(goal_vct);

  ackermann_msgs::AckermannDrive ackermann_cmd;
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;

  if (goal_index != -1)
  {
    ROS_INFO("Goal index: %x", goal_index);

    double l = dist_vct[goal_index];
    geometry_msgs::PoseStamped target_pose = path_.poses[goal_index];
    ROS_INFO("curr_x: %f, curr_y: %f, curr_yaw: %f", curr_x, curr_y, curr_yaw);

    // Find the curvature
    std::tuple<double, double, double> goal_angles = quaternionToEulerAngles1(target_pose.pose.orientation);

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
  else
  {
    setStopCondition(true);
  }
  // Set control_command values as needed
  ackermann_pub_.publish(ackermann_cmd);
}

void PurePursuitTracker::stop()
{
  setStopCondition(false);


  // Send stop command
  ackermann_msgs::AckermannDrive ackermann_cmd;
  ackermann_cmd.speed = 0.0;
  ackermann_cmd.steering_angle = 0.0;
  ackermann_pub_.publish(ackermann_cmd);
}

bool PurePursuitTracker::hasPath()
{
  return path_.poses.size() > 2;
}
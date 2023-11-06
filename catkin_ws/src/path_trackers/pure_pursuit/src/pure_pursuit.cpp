#include <pure_pursuit/pure_pursuit.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <ros/package.h>

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PurePursuit::PurePursuit()
{
  // Initialize ROS node
  ros::NodeHandle nh;

  odom_subscriber_ = nh.subscribe("/gem/base_footprint/odom", 1, &PurePursuit::odomCallback, this);
  gps_subscriber_ = nh.subscribe("/path", 1, &PurePursuit::pathCallback, this);

  ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
}

std::tuple<double, double, double> PurePursuit::quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion)
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

void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  odom_.pose = odom_msg->pose;
}

void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ROS_INFO("Receiving information in path message");
  // path_ = *path_msg;
  path_vct_ = path_msg->poses;

  for (geometry_msgs::PoseStamped pose_stp : path_vct_)
  {
    double x = pose_stp.pose.position.x;
    double y = pose_stp.pose.position.y;

    double qx = pose_stp.pose.orientation.x;
    double qy = pose_stp.pose.orientation.y;
    double qz = pose_stp.pose.orientation.z;
    double qw = pose_stp.pose.orientation.w;

    // ROS_INFO("X: %f, Y: %f", x, y);
    // ROS_INFO("RXQ: %f, RYQ: %f, RZQ: %f, RWQ: %f", qx, qy, qz, qw);
  }
}

double PurePursuit::find_angle(const std::vector<double>& v1, const std::vector<double>& v2)
{
  // Calculate the angle between two vectors v1 and v2
  double dot_product = v1[0] * v2[0] + v1[1] * v2[1];
  double magnitude_v1 = std::sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
  double magnitude_v2 = std::sqrt(v2[0] * v2[0] + v2[1] * v2[1]);
  return std::acos(dot_product / (magnitude_v1 * magnitude_v2));
}


void PurePursuit::process()
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

  // I need to tune waypoints or disntances, because I have a lot of dinstances between the elements.
  // ROS_INFO("Distance found: %f", dist_vct[idx]);
  std::vector<int> goal_vct;
  for (int i = 0; i < dist_vct.size(); i++)
  {
    // ROS_INFO("Distance-i: %f", dist_vct[i]);
    // ROS_INFO("Minor: %f", look_ahead_dist_ - 3.0);
    // ROS_INFO("Major: %f", look_ahead_dist_ + 3.0);

    if (dist_vct[i] >= look_ahead_dist_ - 2.0 && dist_vct[i] <= look_ahead_dist_ + 2.0)
    {
      // ROS_INFO("TRUE");

      // Store the index of the matching element
      goal_vct.push_back(i);
    }
  }

  ROS_INFO("Goals processed: %d", goal_vct.size());

  int goal = -1;
  for (int idx : goal_vct)
  {
    std::vector<double> v1 = {dist_vct[idx] - curr_x, dist_vct[idx] - curr_y};
    std::vector<double> v2 = {std::cos(curr_yaw), std::sin(curr_yaw)};

    double temp_angle = find_angle(v1, v2);

    if (std::abs(temp_angle) < M_PI / 2)
    {
      goal = idx;
      break;
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
    ROS_INFO("GX: %f, GY: %f", target_pose.pose.position.x, target_pose.pose.position.y);
    ROS_INFO("GXQ: %f, GYQ: %f, GZQ: %f, GWQ: %f", target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);

    double xc = target_pose.pose.position.x - curr_x;
    double yc = target_pose.pose.position.y - curr_y;

    // Find the curvature
    std::tuple<double, double, double> goal_angles = quaternionToEulerAngles(target_pose.pose.orientation);

    double goal_yaw = std::get<2>(goal_angles);
    ROS_INFO("Goal yaw: %f", goal_yaw);

    double alpha = goal_yaw - curr_yaw;
    ROS_INFO("Alpha: %f", alpha);

    double k = 0.285;
    double wheelbase = 1.75;
    double angle_i = std::atan((2 * k * wheelbase * std::sin(alpha)) / l);
    ROS_INFO("Angle i: %f", angle_i);

    double angle = angle_i * 2;

    double ct_error = std::sin(alpha) * l;

    ROS_INFO("CTE: %f", ct_error);

    // Publish control commands
    ackermann_cmd.speed = 1.5;
    ackermann_cmd.steering_angle = angle;
  }
  ROS_INFO("Publishing velocity");
  // Set control_command values as needed
  ackermann_pub_.publish(ackermann_cmd);
}


void PurePursuit::run()
{
  ros::Rate rate(1);

  ROS_INFO("In run()");
  while (ros::ok())
  {
    if (path_vct_.size() > 0)
    {
      process();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit path_tracker;

    // Start the path-tracking node
    path_tracker.run();

    return 0;
}
#include <stanley/stanley.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

Stanley::Stanley()
{
  ros::NodeHandle nh;

  odom_subscriber_ = nh.subscribe("/gem/base_footprint/odom", 1, &Stanley::odomCallback, this);
  gps_subscriber_ = nh.subscribe("/path", 1, &Stanley::pathCallback, this);

  ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
}

void Stanley::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  odom_.pose = odom_msg->pose;
}

void Stanley::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ROS_INFO("Receiving information in path message");
  path_vct_ = path_msg->poses;
}

void Stanley::run()
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


std::tuple<double, double, double> Stanley::quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion)
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

double Stanley::pi_2_pi(double angle)
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

void Stanley::process()
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

    ROS_INFO("Distance: %f", dist);

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

  // double part1 = 0.45 * cte;
  // double part2 = f_vel;
  // ROS_INFO("Part1: %f, Part2: %f", part1, part2);

  double partial = std::atan2(0.45 * cte, f_vel);
  double delta = theta_e + partial;
  // std::cout << partial << std::endl;

  ROS_INFO("Delta: %f", delta);
  ROS_INFO("Partial: %f", partial);

  double theta_e_deg = theta_e * (180.0 / M_PI);
  ROS_INFO("CTE: %f, Heading error: %f", cte, theta_e_deg);

  // Publish control commands
  ackermann_msgs::AckermannDrive ackermann_cmd;
  ackermann_cmd.speed = 1.5;
  ackermann_cmd.steering_angle = delta - 1.570797;
  ackermann_pub_.publish(ackermann_cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stanley_node");
  Stanley path_tracker;

  path_tracker.run();

  return 0;
}

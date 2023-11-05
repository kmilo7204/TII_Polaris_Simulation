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

  // self.rate       = rospy.Rate(20)
  // self.look_ahead = 6    # meters
  // self.wheelbase  = 1.75 # meters
  // self.goal       = 0
  // self.read_waypoints() # read waypoints
  // self.ackermann_msg = AckermannDrive()
  // self.ackermann_msg.steering_angle_velocity = 0.0
  // self.ackermann_msg.acceleration            = 0.0
  // self.ackermann_msg.jerk                    = 0.0
  // self.ackermann_msg.speed                   = 0.0
}

std::tuple<double, double, double> PurePursuit::quaternionToEulerAngles(const geometry_msgs::Quaternion& quaternion)
{
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

    std::tuple<double, double, double> { roll, pitch, yaw };
}

void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  // ROS_INFO("Receiving information in odom message");
  // double x_pos = odom_msg->pose.pose.position.x;
  // double y_pos = odom_msg->pose.pose.position.y;
  // ROS_INFO("Printing position. X:%f, Y:%f", x_pos, y_pos);
  odom_.pose = odom_msg->pose;
}

void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ROS_INFO("Receiving information in path message");
  path_ = *path_msg;
  path_vct_ = path_msg->poses;
}

// void PurePursuit:dist();


void PurePursuit::process()
{
  std::tuple<double, double, double> euler_angles = quaternionToEulerAngles(odom_.pose.pose.orientation);

  // get current position and orientation in the world frame
  // curr_x, curr_y, curr_yaw = self.get_gem_pose()
  double curr_x = odom_.pose.pose.position.x;
  double curr_y = odom_.pose.pose.position.y;
  double curr_yaw = std::get<2>(euler_angles);

  std::vector<double> dist_vct;
  dist_vct.reserve(path_vct_.size();)
  int idx = 0;

  // Since we dont know what is the lower distance, we need to find it out.
  for (const geometry_msgs::PoseStamped& pose_stamped : path_vct_)
  {
    pose_stamped.pose.position.x;
    pose_stamped.pose.position.y;
    dist_vct[idx] = sqrt(pow(pose_stamped.pose.position.x - curr_x, 2) + pow(pose_stamped.pose.position.y - curr_y, 2));
    idx++;
  }

  std::vector<int> goal_vct;
  for (int i = 0; i < dist_vct.size(); i++)
  {
    if (dist_vct[i] >= look_ahead_dist_ - 3.0 && dist_vct[i] <= look_ahead_dist_ + 3.0)
    {
      // Store the index of the matching element
      goal_vct.push_back(i);
    }
  }

  for (int idx : goal_arr)
  {
    std::vector<double> v1 = {dist_vct[idx] - curr_x, dist_vct[idx] - curr_y};
    std::vector<double> v2 = {std::cos(curr_yaw), std::sin(curr_yaw)};

    double temp_angle = find_angle(v1, v2);

    if (std::abs(temp_angle) < M_PI / 2) {
        goal = idx;
        break;
    }
  }


//             # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
//             goal_arr = np.where( (self.dist_arr < self.look_ahead + 0.3) & (self.dist_arr > self.look_ahead - 0.3) )[0]

//             # finding the goal point which is the last in the set of points less than the lookahead distance
//             for idx in goal_arr:
//                 v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
//                 v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
//                 temp_angle = self.find_angle(v1,v2)
//                 if abs(temp_angle) < np.pi/2:
//                     self.goal = idx
//                     break

//             # finding the distance between the goal point and the vehicle
//             # true look-ahead distance between a waypoint and current position
//             L = self.dist_arr[self.goal]

//             # transforming the goal point into the vehicle coordinate frame 
//             gvcx = self.path_points_x[self.goal] - curr_x
//             gvcy = self.path_points_y[self.goal] - curr_y
//             goal_x_veh_coord = gvcx*np.cos(curr_yaw) + gvcy*np.sin(curr_yaw)
//             goal_y_veh_coord = gvcy*np.cos(curr_yaw) - gvcx*np.sin(curr_yaw)

//             # find the curvature and the angle 
//             alpha   = self.path_points_yaw[self.goal] - (curr_yaw)
//             k       = 0.285
//             angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / L) 
//             angle   = angle_i*2
//             angle   = round(np.clip(angle, -0.61, 0.61), 3)

//             ct_error = round(np.sin(alpha) * L, 3)

//             print("Crosstrack Error: " + str(ct_error))

//             # implement constant pure pursuit controller
//             self.ackermann_msg.speed          = 2.8
//             self.ackermann_msg.steering_angle = angle
//             self.ackermann_pub.publish(self.ackermann_msg)

//             self.rate.sleep()
}



void PurePursuit::run()
{
  ros::Rate rate(10);
  // readWaypoints();

  ROS_INFO("In run()");
  while (ros::ok())
  {
      // Publish control commands
      ackermann_msgs::AckermannDrive ackermann_cmd;
      // This is working
      ackermann_cmd.speed = 5.1;
      // ROS_INFO("Publishing velocity");
      // Set control_command values as needed
      ackermann_pub_.publish(ackermann_cmd);

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
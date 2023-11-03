#include <pure_pursuit/pure_pursuit.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <ros/package.h>

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
  std::vector<geometry_msgs::Pose> poses_vct;
  for (const geometry_msgs::PoseStamped& pose : path_msg->poses)
  {
    poses_vct.push_back(pose.pose);
    // path_points_x_.push_back(std::get<0>(point));
    // path_points_y_.push_back(std::get<1>(point));
    // path_points_yaw_.push_back(std::get<2>(point));
  }
  ROS_INFO("Receiving information in path message");
  path_msg->poses;
}

// void PurePursuit::readWaypoints()
// {
//   std::string filename = ros::package::getPath("pure_pursuit") + "/src/wps.csv";
//   ROS_INFO("Filename: %s", filename.c_str());
//   std::ifstream file(filename);

//   if (!file.is_open())
//   {
//     std::cerr << "Error: Unable to open waypoints file " << filename << std::endl;
//     return;
//   }

//   std::vector<std::tuple<double, double, double>> path_points;
//   std::string line;

//   while (std::getline(file, line))
//   {
//     std::istringstream ss(line);
//     double x, y, yaw;
//     if (ss >> x >> y >> yaw)
//     {
//       path_points.emplace_back(x, y, yaw);
//     }
//   }

//   // Turn path_points into separate arrays
//   path_points_x_.clear();
//   path_points_y_.clear();
//   path_points_yaw_.clear();
//   dist_arr_.resize(path_points.size(), 0.0);

//   for (const auto& point : path_points)
//   {
//     ROS_INFO("Read point: x=%f, y=%f, yaw=%f", std::get<0>(point), std::get<1>(point), std::get<2>(point));
//     path_points_x_.push_back(std::get<0>(point));
//     path_points_y_.push_back(std::get<1>(point));
//     path_points_yaw_.push_back(std::get<2>(point));
//   }
// }

void PurePursuit::process()
{



// # get current position and orientation in the world frame
//             curr_x, curr_y, curr_yaw = self.get_gem_pose()

//             self.path_points_x = np.array(self.path_points_x)
//             self.path_points_y = np.array(self.path_points_y)

//             # finding the distance of each way point from the current position
//             for i in range(len(self.path_points_x)):
//                 self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

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
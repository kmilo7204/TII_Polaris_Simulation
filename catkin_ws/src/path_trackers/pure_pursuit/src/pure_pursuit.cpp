#include <pure_pursuit/pure_pursuit.hpp>

PurePursuit::PurePursuit()
{
  // Initialize ROS node
  ros::NodeHandle nh;

  odom_subscriber_ = nh.subscribe("path_topic", 1, &PurePursuit::odomCallback, this);
  gps_subscriber_ = nh.subscribe("gps_topic", 1, &PurePursuit::gpsCallback, this);

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

void PurePursuit::odomCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ;
}

void PurePursuit::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
  ;
}

void PurePursuit::run()
{
  ros::Rate rate(10);

  while (ros::ok())
  {
      // Publish control commands
      ackermann_msgs::AckermannDrive ackermann_cmd;
      ROS_INFO("Publishing velocity");
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
#include <gtest/gtest.h>

#include <iostream>

#include <ros/ros.h>

#include <path_generator/path_generator.hpp>

struct PathGeneratorHelper
{
    PathGeneratorHelper()
        : path_size(0)
    {
    }

    void cb(const nav_msgs::Path::ConstPtr &msg)
    {
        path_size = msg->poses.size();
    }

    uint32_t path_size;
};

TEST(PathGeneratorTest, SuccessfulReadAndPublishPathService)
{
    // Instantiate the object
    PathGenerator path_generator;

    // Create the request and response objects
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;

    // Call the service to read waypoints from CSV
    path_generator.readAndPublishPathSvcCallback(req, res);

    // Service response was successful
    EXPECT_EQ(res.success, true);
    // CSV file contains valid data
    EXPECT_GT(path_generator.getPath().poses.size(), 0);
}

TEST(PathGeneratorTest, PublishPathish)
{
    // Instantiate the object
    PathGenerator path_generator;

    // Reuse previous test, to make sure we covered that case in the whole flow
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    path_generator.readAndPublishPathSvcCallback(req, res);
    EXPECT_GT(path_generator.getPath().poses.size(), 0);

    // Create a Nodehandler to subscribe to the path
    PathGeneratorHelper path_gen_helper;

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/path", 0, &PathGeneratorHelper::cb, &path_gen_helper);

    // Set up a ROS timer to allow time for reading subscription
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent &) {});

    // Publish the path
    path_generator.publishPath();

    // Wait for a short duration to allow the timer to expire
    ros::Duration(2.0).sleep();

    // Spin the test node to process callbacks
    ros::spinOnce();

    // Stop the timer
    timer.stop();

    // Check that we received the test message
    EXPECT_GT(path_gen_helper.path_size, 1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node");
    return RUN_ALL_TESTS();
}

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <tuple>

#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


std::tuple<double, double, double> quaternionToEulerAngles1(const geometry_msgs::Quaternion& quaternion)
{
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    return std::make_tuple(roll, pitch, yaw);
}

double angleBetweenVectors(const std::vector<double>& v1, const std::vector<double>& v2)
{
  // Calculate the angle between two vectors v1 and v2
  double dot_product = v1[0] * v2[0] + v1[1] * v2[1];
  double magnitude_v1 = std::sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
  double magnitude_v2 = std::sqrt(v2[0] * v2[0] + v2[1] * v2[1]);
  return std::acos(dot_product / (magnitude_v1 * magnitude_v2));
}

#endif

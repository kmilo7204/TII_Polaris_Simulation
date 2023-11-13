// test_utils.cpp
#include <gtest/gtest.h>
#include <utils/utils.hpp>

TEST(UtilsTest, QuaternionToEulerAngles)
{
  geometry_msgs::Quaternion quaternion;
  quaternion.w = 1.0;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 0.0;

  auto angles = quaternionToEulerAngles(quaternion);

  EXPECT_DOUBLE_EQ(std::get<0>(angles), 0.0);  // Roll
  EXPECT_DOUBLE_EQ(std::get<1>(angles), 0.0);  // Pitch
  EXPECT_DOUBLE_EQ(std::get<2>(angles), 0.0);  // Yaw
}

TEST(UtilsTest, AngleBetweenVectors)
{
  std::vector<double> v1 = {1.0, 0.0};
  std::vector<double> v2 = {0.0, 1.0};

  double angle = angleBetweenVectors(v1, v2);

  // For orthogonal vectors, the angle should be PI/2
  EXPECT_DOUBLE_EQ(angle, M_PI / 2.0);
}

TEST(UtilsTest, PiToPi)
{
  double angle1 = 3.0 * M_PI;
  double angle2 = -3.0 * M_PI;
  double angle3 = 1.0;

  double result1 = piToPi(angle1);
  double result2 = piToPi(angle2);
  double result3 = piToPi(angle3);

  // Angle to be adjusted to PI = 3PI - 2PI
  EXPECT_DOUBLE_EQ(result1, M_PI);
  // Angle to be adjusted to PI = -3PI + 2PI
  EXPECT_DOUBLE_EQ(result2, -M_PI);
  // No adjustment
  EXPECT_DOUBLE_EQ(result3, angle3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

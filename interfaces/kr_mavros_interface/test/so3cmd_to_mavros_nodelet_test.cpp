#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <kr_mav_msgs/SO3Command.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <thread>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <kr_mav_msgs/Corrections.h>
#include "kr_mavros_interface/so3cmd_to_mavros_testor.hpp"

/**
 * @brief Test1: baseline test
 * Expected behavior: attitude_raw_msg_received = false, odom_msg_received = false
 */
TEST(SO3CmdToMavrosTester, Test1)
{
  std::cout << "Performing Test1!\n";
  SO3CmdToMavrosTester tester;
  {
    std::lock_guard<std::mutex> lock(tester.mutex);
    EXPECT_FALSE(tester.attitude_raw_msg_received_);
    EXPECT_FALSE(tester.odom_msg_received_);
  }
}

/**
 * @brief Test2: publish odom message and check if it is received
 * Expected behavior: odom_msg_received = true, odom_msg_.pose.pose = odom.pose.pose
 */
TEST(SO3CmdToMavrosTester, Test2)
{
  std::cout << "Performing Test2!\n";
  SO3CmdToMavrosTester tester;
  tester.publish_odom_msg();
  ros::Duration(1.0).sleep();
  {
    std::lock_guard<std::mutex> lock(tester.mutex);
    EXPECT_TRUE(tester.is_odom_publisher_active());
    EXPECT_TRUE(tester.odom_msg_received_);
    EXPECT_EQ(tester.odom_msg_.pose.pose.position.x, 0.0);
    EXPECT_EQ(tester.odom_msg_.pose.pose.position.y, 0.0);
    EXPECT_EQ(tester.odom_msg_.pose.pose.position.z, 0.0);
    EXPECT_EQ(tester.odom_msg_.pose.pose.orientation.w, 1.0);
    EXPECT_EQ(tester.odom_msg_.pose.pose.orientation.x, 0.0);
    EXPECT_EQ(tester.odom_msg_.pose.pose.orientation.y, 0.0);
    EXPECT_EQ(tester.odom_msg_.pose.pose.orientation.z, 0.0);
  }
}

/**
 * @brief Test3: publish odom message and imu message and so3 command with motor disabled
 * Expected behavior: attitude_raw_msg_received = true, odom_msg_received = true
 */
TEST(SO3CmdToMavrosTester, Test3)
{
  std::cout << "Performing Test3!\n"; 
  SO3CmdToMavrosTester tester;
  tester.populate_so3cmd_vector();
  tester.publish_odom_msg();
  tester.publish_imu_msg();
  ros::Duration(1.0).sleep();
  tester.publish_so3_cmd(0);
  {
    std::lock_guard<std::mutex> lock(tester.mutex);
    EXPECT_TRUE(tester.is_imu_publisher_active());
    EXPECT_TRUE(tester.is_so3cmd_publisher_active());
    ros::Duration(5.0).sleep();
    EXPECT_TRUE(tester.attitude_raw_msg_received_);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.w, 1.0);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.x, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.y, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.z, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.body_rate.x, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.body_rate.y, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.body_rate.z, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.thrust, 0.0);
  }
}

/**
 * @brief Test4: publish odom message and imu message and so3 command with motor enabled
 * Expected behavior: attitude_raw_msg_received = true, odom_msg_received = true
 */
TEST(SO3CmdToMavrosTester, Test4)
{
  std::cout << "Performing Test4!\n"; 
  SO3CmdToMavrosTester tester;
  tester.populate_so3cmd_vector();
  tester.publish_odom_msg();
  tester.publish_imu_msg();
  ros::Duration(1.0).sleep();
  tester.publish_so3_cmd(1);
  {
    std::lock_guard<std::mutex> lock(tester.mutex);
    EXPECT_TRUE(tester.is_imu_publisher_active());
    EXPECT_TRUE(tester.is_so3cmd_publisher_active());
    ros::Duration(5.0).sleep();
    EXPECT_TRUE(tester.attitude_raw_msg_received_);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.w, 1.0);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.x, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.y, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.orientation.z, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.body_rate.x, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.body_rate.y, 0.0);
    EXPECT_EQ(tester.attitude_raw_msg_.body_rate.z, 0.0);
    EXPECT_NEAR(tester.attitude_raw_msg_.thrust, 0.1, 1e-4);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "so3_interface_nodelet_test");
  testing::InitGoogleTest(&argc, argv);

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  t.join();

  return res;
}


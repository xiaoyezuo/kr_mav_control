#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <kr_mav_msgs/SO3Command.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <thread>
#include <mutex>
// #include "kr_mav_controllers/so3_control_tester.hpp"
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


TEST(SO3CmdToMavrosTester, Test1)
{
  std::cout << "Performing Test1!\n";
  SO3CmdToMavrosTester tester;
  {
    std::lock_guard<std::mutex> lock(tester.mutex);
    EXPECT_FALSE(tester.attitude_raw_msg_received_);
    EXPECT_FALSE(tester.odom_msg_received_);
  }
  ASSERT_TRUE(tester.is_odom_publisher_active());  // checking if nodelet is active
  ASSERT_TRUE(tester.is_imu_publisher_active());  // checking if nodelet is active
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "so3_interface_nodelet_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


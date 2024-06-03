#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <kr_mav_msgs/SO3Command.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

class SO3CmdToMavros : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_;
  int num_props_;

  ros::Publisher attitude_raw_pub_;
  ros::Publisher odom_pose_pub_;  // For sending PoseStamped to firmware

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;

  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  kr_mav_msgs::SO3Command last_so3_cmd_;
};

TEST(SO3CmdToMavros, odom_callback)
{
  SO3CmdToMavros so3_cmd_to_mavros;
  nav_msgs::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  so3_cmd_to_mavros.odom_callback(&odom);
  EXPECT_TRUE(so3_cmd_to_mavros.odom_set_);
}

TEST(SO3CmdToMavros, imu_callback)
{
  SO3CmdToMavros so3_cmd_to_mavros;
  sensor_msgs::Imu imu;
  imu.orientation.w = 1.0;
  imu.orientation.x = 0.0;
  imu.orientation.y = 0.0;
  imu.orientation.z = 0.0;
  so3_cmd_to_mavros.imu_callback(&imu);
  EXPECT_TRUE(so3_cmd_to_mavros.imu_set_);
}

TEST(SO3CmdToMavros, so3_cmd_callback)
{
  SO3CmdToMavros so3_cmd_to_mavros;
  kr_mav_msgs::SO3Command so3_cmd;
  so3_cmd_to_mavros.so3_cmd_callback(&so3_cmd);
  EXPECT_TRUE(so3_cmd_to_mavros.so3_cmd_set_);
}
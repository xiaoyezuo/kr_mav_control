#include <kr_mav_msgs/Corrections.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_mav_msgs/SO3Command.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>

// Test class for SO3CmdToMavros nodelet
class SO3CmdToMavrosTester
{
 public:
  SO3CmdToMavrosTester();
  void publish_so3_cmd();
  void publish_odom_msg();
  void publish_imu_msg();
  void attitude_raw_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
  void odom_callback(const geometry_msgs::PoseStamped::ConstPtr &odom);
  bool is_imu_publisher_active();
  bool is_odom_publisher_active();
  bool is_so3cmd_publisher_active();
  bool attitude_raw_msg_received_ = false;
  bool odom_msg_received_ = false;
  mavros_msgs::AttitudeTarget attitude_raw_msg_;
  nav_msgs::Odometry odom_msg_;
  std::mutex mutex;

  // bool odom_set_, imu_set_, so3_cmd_set_;
  // Eigen::Quaterniond odom_q_, imu_q_;
  // double kf_, lin_cof_a_, lin_int_b_;
  // int num_props_;
  // double so3_cmd_timeout_;
  // ros::Time last_so3_cmd_time_;
  // kr_mav_msgs::SO3Command last_so3_cmd_;

  private:
    ros::NodeHandle nh;
    ros::Publisher so3_cmd_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_pub_;
    ros::Subscriber attitude_raw_sub_;
    ros::Subscriber odom_pose_sub_;  // For sending PoseStamped to firmware
};

// Initialize the nodelet
SO3CmdToMavrosTester::SO3CmdToMavrosTester(): nh("")
{
  // ros::NodeHandle nh;
  // nh.getParam("kf", kf_);
  // nh.getParam("num_props", num_props_);
  so3_cmd_pub_ = nh.advertise<kr_mav_msgs::SO3Command>("so3_cmd", 5, true);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 5, true);
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("mavros/imu/data", 5, true);
  attitude_raw_sub_ = nh.subscribe("attitude_raw", 5, &SO3CmdToMavrosTester::attitude_raw_callback, this, ros::TransportHints().tcpNoDelay());
  odom_pose_sub_ = nh.subscribe("mavros/vision_pose/pose", 5, &SO3CmdToMavrosTester::odom_callback, this, ros::TransportHints().tcpNoDelay());
}

// Check if so3cmd publisher is active
bool SO3CmdToMavrosTester::is_so3cmd_publisher_active()
{
  return so3_cmd_pub_.getNumSubscribers() > 0;
}

//check if imu publisher is active
bool SO3CmdToMavrosTester::is_imu_publisher_active()
{
  return imu_pub_.getNumSubscribers() > 0;
}

// Check if odom publisher is active
bool SO3CmdToMavrosTester::is_odom_publisher_active()
{
  return odom_pub_.getNumSubscribers() > 0;
}

// Publish so3_cmd message
void SO3CmdToMavrosTester::publish_so3_cmd()
{
  ROS_INFO("Publishing SO3 Command message to %s", so3_cmd_pub_.getTopic().c_str());
  kr_mav_msgs::SO3Command so3_cmd;
  so3_cmd.header.stamp = ros::Time::now();
  so3_cmd.force.x = 0.0;
  so3_cmd.force.y = 0.0;
  so3_cmd.force.z = 0.0;
  so3_cmd.orientation.w = 1.0;
  so3_cmd.orientation.x = 0.0;
  so3_cmd.orientation.y = 0.0;
  so3_cmd.orientation.z = 0.0;
  so3_cmd.angular_velocity.x = 0.0;
  so3_cmd.angular_velocity.y = 0.0;
  so3_cmd.angular_velocity.z = 0.0;
  so3_cmd.aux.current_yaw = 0.0;
  so3_cmd.aux.kf_correction = 0.0;
  so3_cmd.aux.angle_corrections[0] = 0.0;
  so3_cmd.aux.angle_corrections[1] = 0.0;
  so3_cmd_pub_.publish(so3_cmd);
}

// Publish odom message
void SO3CmdToMavrosTester::publish_odom_msg()
{
  ROS_INFO("Publishing Odom message to %s", odom_pub_.getTopic().c_str());
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom_pub_.publish(odom);
}

// Publish imu message
void SO3CmdToMavrosTester::publish_imu_msg()
{
  ROS_INFO("Publishing IMU message to %s", imu_pub_.getTopic().c_str());
  sensor_msgs::Imu imu;
  imu.orientation.w = 1.0;
  imu.orientation.x = 0.0;
  imu.orientation.y = 0.0;
  imu.orientation.z = 0.0;
  imu_pub_.publish(imu);
}

// Callback function for attitude_raw subscriber
void SO3CmdToMavrosTester::attitude_raw_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
  ROS_INFO("Received Attitude Raw message from %s", attitude_raw_sub_.getTopic().c_str());
  std::lock_guard<std::mutex> lock(mutex);
  {
    attitude_raw_msg_received_ = true;
    if(attitude_raw_msg_received_){
      attitude_raw_msg_ = *msg;
    }
  }
}

// Callback function for odom_pose subscriber
void SO3CmdToMavrosTester::odom_callback(const geometry_msgs::PoseStamped::ConstPtr &odom)
{
  ROS_INFO("Received Odom message from %s", odom_pose_sub_.getTopic().c_str());
  std::lock_guard<std::mutex> lock(mutex);
  odom_msg_received_ = true;
  odom_msg_.pose.pose = odom->pose;
}

// struct Test4Data
// {
//   float position_x = 0.0;
//   float position_y = 0.0;
//   float position_z = 0.0;
//   float orientation_w = 1.0;
//   float orientation_x = 0.0;
//   float orientation_y = 0.0;
//   float orientation_z = 0.0;
// };





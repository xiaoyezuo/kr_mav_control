#include <kr_mav_msgs/Corrections.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_mav_msgs/SO3Command.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>
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

class SO3CmdToMavrosTester
{
 public:
  void onInit(void);
  void publish_so3_cmd();
  void publsh_odom_msg();
  void publish_imu_msg();
  void attitude_raw_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  bool is_odom_publisher_active();
  bool is_imu_publisher_active();
  bool attitude_raw_msg_received_ = false;
  bool odom_msg_received_ = false;
  mavros_msgs::AttitudeTarget attitude_raw_msg_;
  nav_msgs::Odometry odom_msg_;
  std::mutex mutex;

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_;
  int num_props_;
  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  kr_mav_msgs::SO3Command last_so3_cmd_;

  private:
    ros::Publisher so3_cmd_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_pub_;
    ros::Subscriber attitude_raw_sub_;
    ros::Subscriber odom_pose_sub_;  // For sending PoseStamped to firmware
};

void SO3CmdToMavrosTester::onInit(void)
{
  ros::NodeHandle nh;
  so3_cmd_pub_ = nh.advertise<kr_mav_msgs::SO3Command>("so3_cmd", 1);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 1);
  attitude_raw_sub_ = nh.subscribe<mavros_msgs::AttitudeTarget>("attitude_raw", 1, &SO3CmdToMavrosTester::attitude_raw_callback, this);
  odom_pose_sub_ = nh.subscribe<nav_msgs::Odometry>("odom_pose", 1, &SO3CmdToMavrosTester::odom_callback, this);
}

bool SO3CmdToMavrosTester::is_odom_publisher_active()
{
  return odom_pub_.getNumSubscribers() > 0;
}

bool SO3CmdToMavrosTester::is_imu_publisher_active()
{
  return imu_pub_.getNumSubscribers() > 0;
}

void SO3CmdToMavrosTester::publish_so3_cmd()
{
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

void SO3CmdToMavrosTester::publsh_odom_msg()
{
  nav_msgs::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom_pub_.publish(odom);
}

void SO3CmdToMavrosTester::publish_imu_msg()
{
  sensor_msgs::Imu imu;
  imu.orientation.w = 1.0;
  imu.orientation.x = 0.0;
  imu.orientation.y = 0.0;
  imu.orientation.z = 0.0;
  imu_pub_.publish(imu);
}

void SO3CmdToMavrosTester::attitude_raw_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
  ROS_INFO("Attitude Raw Callback");
  attitude_raw_msg_ = *msg;
}

void SO3CmdToMavrosTester::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  ROS_INFO("Odom Callback");
  odom_msg_ = *odom;
}






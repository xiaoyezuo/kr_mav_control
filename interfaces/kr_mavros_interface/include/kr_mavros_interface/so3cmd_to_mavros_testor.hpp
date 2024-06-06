#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include "kr_mav_msgs/msg/so3_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/logger.hpp"

/**
 * @brief Test class for SO3CmdToMavros nodelet
 */
class SO3CmdToMavrosTester : public rclcpp::Node
{
 public:
  SO3CmdToMavrosTester();
  void populate_so3cmd_vector();
  void publish_so3_cmd(int idx);
  void publish_odom_msg();
  void publish_imu_msg();
  void attitude_raw_callback(const mavros_msgs::msg::AttitudeTarget::SharedPtr msg);
  void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr odom);
  bool is_imu_publisher_active();
  bool is_odom_publisher_active();
  bool is_so3cmd_publisher_active();
  bool attitude_raw_msg_received_ = false;
  bool odom_msg_received_ = false;
  mavros_msgs::msg::AttitudeTarget::SharedPtr attitude_raw_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr odom_msg_;
  std::mutex mutex;

  private:
    rclcpp::Publisher<kr_mav_msgs::msg::SO3Command>::SharedPtr so3_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_sub_;
    std::vector<kr_mav_msgs::msg::SO3Command> so3_cmd_msgs_;
    std::vector<nav_msgs::msg::Odometry> odom_msgs_;
    std::vector<sensor_msgs::msg::Imu> imu_msgs_;
};

/**
 * @brief Construct a new SO3CmdToMavrosTester::SO3CmdToMavrosTester object
 */
SO3CmdToMavrosTester::SO3CmdToMavrosTester(): Node("so3cmd_to_mavros_tester")
{
    so3_cmd_pub_ = this->create_publisher<kr_mav_msgs::msg::SO3Command>("so3_cmd", 5);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("mavros/imu/data", 5);
    attitude_raw_sub_ = this->create_subscription<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/attitude", 5, std::bind(&SO3CmdToMavrosTester::attitude_raw_callback, this, std::placeholders::_1));
    odom_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/vision_pose/pose", 5, std::bind(&SO3CmdToMavrosTester::odom_callback, this, std::placeholders::_1));
}

/**
 * @brief Check if so3_cmd publisher is active
 * 
 */
bool SO3CmdToMavrosTester::is_so3cmd_publisher_active()
{
  return so3_cmd_pub_->get_subscription_count() > 0;
}

/**
 * @brief Check if imu publisher is active
 * 
 */
bool SO3CmdToMavrosTester::is_imu_publisher_active()
{
  return imu_pub_->get_subscription_count() > 0;
}

/**
 * @brief Check if odom publisher is active
 * 
 */
bool SO3CmdToMavrosTester::is_odom_publisher_active()
{
  return odom_pub_->get_subscription_count() > 0;
}

/**
 * @brief Populates the so3_cmd_msgs_ vector with SO3Command messages
 * 
 */
void SO3CmdToMavrosTester::populate_so3cmd_vector()
{
  //enable_motors = false
  auto so3_cmd = kr_mav_msgs::msg::SO3Command();
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
  so3_cmd.aux.enable_motors = false;
  so3_cmd.aux.current_yaw = 0.0;
  so3_cmd.aux.kf_correction = 0.0;
  so3_cmd.aux.angle_corrections[0] = 0.0;
  so3_cmd.aux.angle_corrections[1] = 0.0;
  so3_cmd_msgs_.push_back(so3_cmd);

  //enable_motors = true
  so3_cmd.aux.enable_motors = true;
  so3_cmd_msgs_.push_back(so3_cmd);

  //enable_motors = true, force = 1.0
  so3_cmd.orientation.w = 1.0;
  so3_cmd.orientation.x = 0.0;
  so3_cmd.orientation.y = 0.0;
  so3_cmd.orientation.z = 0.0;
  so3_cmd.force.x = 1.0;
  so3_cmd.force.y = 1.0;
  so3_cmd.force.z = 1.0;
  so3_cmd_msgs_.push_back(so3_cmd);
}

/**
 * @brief Publish SO3 Command message indexed from so3_cmd_msgs_ vector
 * 
 * @param idx index of the SO3 Command message to publish
 */
void SO3CmdToMavrosTester::publish_so3_cmd(int idx)
{
    // RCLCPP_INFO(this->get_logger(), "Publishing SO3 Command message to %s", so3_cmd_pub_->get_topic_name());
    int so3_cmd_vector_size = so3_cmd_msgs_.size();
    if(idx < so3_cmd_vector_size)
    {
        so3_cmd_pub_->publish(so3_cmd_msgs_[idx]);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Index out of range. Cannot publish SO3 Command message");
    }
}

/**
 * @brief Publish odom message
 */
void SO3CmdToMavrosTester::publish_odom_msg()
{
    // RCLCPP_INFO(this->get_logger(), "Publishing Odom message to %s", odom_pub_->get_topic_name());
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom_pub_->publish(odom);
}

/**
 * @brief Publish IMU message
 */
void SO3CmdToMavrosTester::publish_imu_msg()
{
  // RCLCPP_INFO(this->get_logger(), "Publishing IMU message to %s", imu_pub_->get_topic_name());
  sensor_msgs::msg::Imu imu;
  imu.orientation.w = 1.0;
  imu.orientation.x = 0.0;
  imu.orientation.y = 0.0;
  imu.orientation.z = 0.0;
  imu_pub_->publish(imu);
}

/**
 * @brief Callback function to handle attitude raw messages from nodelet
 * 
 * @param msg message received from attitude_raw_sub_
 */
void SO3CmdToMavrosTester::attitude_raw_callback(const mavros_msgs::msg::AttitudeTarget::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received Attitude Raw message from %s", attitude_raw_sub_->get_topic_name());
    std::lock_guard<std::mutex> lock(mutex);
    attitude_raw_msg_received_ = true;
    attitude_raw_msg_ = std::make_shared<mavros_msgs::msg::AttitudeTarget>(*msg);
}

/**
 * @brief Callback function to handle odom messages from nodelet
 * 
 * @param odom message received from odom_pose_sub_
 */
void SO3CmdToMavrosTester::odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr odom)
{
    // RCLCPP_INFO(this->get_logger(), "Received Odom message from %s", odom_pose_sub_->get_topic_name());
    std::lock_guard<std::mutex> lock(mutex);
    odom_msg_received_ = true;
    odom_msg_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*odom);
}
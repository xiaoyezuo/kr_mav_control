#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "kr_mav_msgs/msg/so3_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "mavros_msgs/msg/attitude_target.hpp"

class SO3CmdToMavros : public rclcpp::Node
{
 public:
    explicit SO3CmdToMavros(const rclcpp::NodeOptions &options);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::UniquePtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom);
  void imu_callback(const sensor_msgs::msg::Imu::UniquePtr pose);
  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_;
  int num_props_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_pub_;  // For sending PoseStamped to firmware

  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr so3_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  double so3_cmd_timeout_;
  rclcpp::Time last_so3_cmd_time_;
  kr_mav_msgs::msg::SO3Command last_so3_cmd_;
};

// Callback function for odometry messages
void SO3CmdToMavros::odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom)
{

  std::cout << "odom_callback\n";
  odom_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  odom_set_ = true;

  // Publish PoseStamped for mavros vision_pose plugin
  auto odom_pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  odom_pose_msg->header = odom->header;
  odom_pose_msg->pose = odom->pose.pose;
  odom_pose_pub_->publish(std::move(odom_pose_msg));
}

// Callback function for IMU messages
void SO3CmdToMavros::imu_callback(const sensor_msgs::msg::Imu::UniquePtr pose)
{
  std::cout << "imu_callback\n";
  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);
  imu_set_ = true;

  if(so3_cmd_set_ && ((this->now() - last_so3_cmd_time_).seconds() >= so3_cmd_timeout_))
  {
    RCLCPP_INFO(this->get_logger(), "so3_cmd timeout. %f seconds since last command", (this->now() - last_so3_cmd_time_).seconds());
    auto last_so3_cmd_ptr = std::make_unique<kr_mav_msgs::msg::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(std::move(last_so3_cmd_ptr));
  }
}

// Callback function for SO3Command messages
void SO3CmdToMavros::so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::UniquePtr msg)
{

  // both imu_q_ and odom_q_ would be uninitialized if not set
  if(!imu_set_)
  {
    RCLCPP_WARN(this->get_logger(), "Did not receive any imu messages from %s", imu_sub_->get_topic_name());
    return;
  }

  if(!odom_set_)
  {
    RCLCPP_WARN(this->get_logger(), "Did not receive any odom messages from %s", odom_sub_->get_topic_name());
    return;
  }

  // transform to take into consideration the different yaw of the flight
  // controller imu and the odom
  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);
  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  // convert to tf2::Quaternion
  tf2::Quaternion imu_tf = tf2::Quaternion(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  tf2::Quaternion odom_tf = tf2::Quaternion(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

  // extract RPY's
  double imu_roll, imu_pitch, imu_yaw;
  double odom_roll, odom_pitch, odom_yaw;
  tf2::Matrix3x3(imu_tf).getRPY(imu_roll, imu_pitch, imu_yaw);
  tf2::Matrix3x3(odom_tf).getRPY(odom_roll, odom_pitch, odom_yaw);
  
  // create only yaw tf2:Quaternions
  tf2::Quaternion imu_tf_yaw;
  tf2::Quaternion odom_tf_yaw;
  imu_tf_yaw.setRPY(0.0, 0.0, imu_yaw);
  odom_tf_yaw.setRPY(0.0, 0.0, odom_yaw);
  const tf2::Quaternion tf_imu_odom_yaw = imu_tf_yaw * odom_tf_yaw.inverse();

  // transform!
  const Eigen::Quaterniond q_des_transformed =
    Eigen::Quaterniond(tf_imu_odom_yaw.w(), tf_imu_odom_yaw.x(), tf_imu_odom_yaw.y(), tf_imu_odom_yaw.z()) * q_des;

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(odom_q_);

  const float Psi = 0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) + R_des(2, 0) * R_cur(2, 0) +
                                    R_des(0, 1) * R_cur(0, 1) + R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
                                    R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) + R_des(2, 2) * R_cur(2, 2)));

  if(Psi > 1.0f)  // Position control stability guaranteed only when Psi < 1
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 100,  "Psi > 1.0, orientation error is too large!");
  }

  double throttle = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);
  // Scale force to individual rotor velocities (rad/s).
  throttle = std::sqrt(throttle / num_props_ / kf_);

  // Scaling from rotor velocity (rad/s) to att_throttle for pixhawk
  throttle = lin_cof_a_ * throttle + lin_int_b_;

  // failsafe for the error in traj_gen that can lead to nan values
  //prevents throttle from being sent to 1 if it is nan.
  if (isnan(throttle))
  {
    throttle = 0.0;
  }

  // clamp from 0.0 to 1.0
  throttle = std::min(1.0, throttle);
  throttle = std::max(0.0, throttle);

  if(!msg->aux.enable_motors)
    throttle = 0;

  // publish messages
  auto setpoint_msg = std::make_unique<mavros_msgs::msg::AttitudeTarget>();
  setpoint_msg->header = msg->header;
  setpoint_msg->type_mask = 0;
  setpoint_msg->orientation.w = q_des_transformed.w();
  setpoint_msg->orientation.x = q_des_transformed.x();
  setpoint_msg->orientation.y = q_des_transformed.y();
  setpoint_msg->orientation.z = q_des_transformed.z();
  setpoint_msg->body_rate.x = msg->angular_velocity.x;
  setpoint_msg->body_rate.y = msg->angular_velocity.y;
  setpoint_msg->body_rate.z = msg->angular_velocity.z;
  setpoint_msg->thrust = throttle;

  attitude_raw_pub_->publish(std::move(setpoint_msg));

  // save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = rclcpp::Time(msg->header.stamp);
  so3_cmd_set_ = true;
}

// Constructor
SO3CmdToMavros::SO3CmdToMavros(const rclcpp::NodeOptions &options)
    : Node("so3_cmd_to_mavros", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  // Load parameters
    this->declare_parameter("kf", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("lin_cof_a", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("lin_int_b", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("so3_cmd_timeout", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter<int>("num_props", 4);

    this->get_parameter("kf", kf_);
    this->get_parameter("lin_cof_a", lin_cof_a_);
    this->get_parameter("lin_int_b", lin_int_b_);
    this->get_parameter("num_props", num_props_);
    this->get_parameter("so3_cmd_timeout", so3_cmd_timeout_);

    // Initialize variables
    odom_set_ = false;
    imu_set_ = false;
    so3_cmd_set_ = false;

    // Print parameters
    std::cout << "kf: " << kf_ << std::endl;
    std::cout << "lin_cof_a: " << lin_cof_a_ << std::endl;
    std::cout << "lin_int_b: " << lin_int_b_ << std::endl;
    std::cout << "num_props: " << num_props_ << std::endl;
    std::cout << "so3_cmd_timeout: " << so3_cmd_timeout_ << std::endl;

  // Initialize subscribers
  so3_cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::SO3Command>("so3_cmd", 1, std::bind(&SO3CmdToMavros::so3_cmd_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&SO3CmdToMavros::odom_callback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&SO3CmdToMavros::imu_callback, this, std::placeholders::_1));

  // Initialize publishers
  attitude_raw_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("attitude_raw", 1);
  odom_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom_pose", 1);

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SO3CmdToMavros)
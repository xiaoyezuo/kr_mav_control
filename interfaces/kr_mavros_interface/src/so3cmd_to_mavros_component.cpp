#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "kr_mav_msgs/msg/so3_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class SO3CmdToMavros : public rclcpp::Node
{
 public:
    explicit SO3CmdToMavros(const rclcpp::NodeOptions &options);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr pose);
  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_;
  int num_props_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr attitude_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_pub_;  // For sending PoseStamped to firmware

  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr so3_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  double so3_cmd_timeout_;
  rclcpp::Time last_so3_cmd_time_;
  kr_mav_msgs::msg::SO3Command last_so3_cmd_;
};

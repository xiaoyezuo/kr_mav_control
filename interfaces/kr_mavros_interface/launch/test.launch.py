from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

  robot_ns = LaunchConfiguration('robot')  
  odom_topic = LaunchConfiguration('odom')
  so3_cmd_topic = LaunchConfiguration('so3_cmd')

  robot_arg = DeclareLaunchArgument(
    'robot', default_value=''
  )
  odom_arg = DeclareLaunchArgument(
    'odom', default_value='odom'
  )
  so3_cmd_arg = DeclareLaunchArgument(
    'so3_cmd', default_value='so3_cmd'
  )

  # Path to the configuration file
  config_file = FindPackageShare('kr_mavros_interface').find('kr_mavros_interface') + '/config/mavros.yaml'

  # Component configuration
  so3cmd_to_mavros_component = ComposableNodeContainer(
    name="so3_container",
    namespace=LaunchConfiguration('robot'),
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
      ComposableNode(
        package="kr_mavros_interface",
        plugin="SO3CmdToMavros",
        name="so3cmd_to_mavros",
        parameters=[config_file],
        remappings=[
          ("~/odom", odom_topic),
          ("~/so3_cmd", so3_cmd_topic),
          ("~/imu", "mavros/imu/data"),
          ("/attitude_raw", "mavros/setpoint_raw/attitude"),
          ("odom_pose", "mavros/vision_pose/pose")
        ]
      )
    ],
    output='screen'
  )

  return LaunchDescription([
      robot_arg,
      odom_arg,
      so3_cmd_arg,
      so3cmd_to_mavros_component
  ])
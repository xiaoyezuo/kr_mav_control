from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch
import unittest

def generate_test_description():

    # initialize component
    component_under_test = ComposableNodeContainer(
        name="so3_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions= [
            ComposableNode(
                package="kr_mavros_interface",
                plugin="SO3CmdToMavros",
                name="so3cmd_to_mavros",
                remappings=[
                    ("odom", "odom"),
                    ("so3_cmd", "so3_cmd"),
                    ("imu", "mavros/imu/data"),
                    ("attitude_raw", "mavros/setpoint_raw/attitude"),
                    ("odom_pose", "mavros/vision_pose/pose")
                ],
                parameters=[get_package_share_directory("kr_mavros_interface") + '/config/mavros.yaml']
            )
        ],
        output="screen"
    )

    # initialize test node
    so3_to_mavros_component_test = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "so3_to_mavros_component_test",
            ]
        ),
        output="screen",
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
        component_under_test,
        so3_to_mavros_component_test,
        launch_testing.actions.ReadyToTest(),
    ]), {'component_under_test': component_under_test,
         'so3_to_mavros_component_test': so3_to_mavros_component_test}

class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, so3_to_mavros_component_test):
        proc_info.assertWaitForShutdown(so3_to_mavros_component_test, timeout=1000.0)

@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes
    def test_gtest_pass(self, proc_info, so3_to_mavros_component_test):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=so3_to_mavros_component_test
        )
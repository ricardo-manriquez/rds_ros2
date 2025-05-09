from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import yaml

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    rds_node = Node(
        package='rds_ros2',
        namespace='',
        executable='rds_ros2',
        name='rds_ros2',
        parameters=[
            {'front_lidar': '/scan_filtered'},
            {'cmd_vel_in': '/cmd_vel'},
            {'cmd_vel_out': '/cmd_vel_internal'},
            {'reference_point_y': 0.177},
            {'capsule_center_front_y': 0.177},
            {'capsule_center_rear_y': -0.229},
            {'capsule_radius': 0.265},
            {'dt': 0.05},
        ],
        output='screen',
    )

    rds_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_rear_lidar_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'tf_rds']
    )

    filternode = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("rds_ros2"),
                "config", "range_filter.yaml",
            ])],
        remappings=[
          ('scan',          '/scan'),
          ('scan_filtered', '/scan_filtered'),
        ],
    )


    return LaunchDescription([
        rds_node,
        rds_frame,
        filternode,
    ])

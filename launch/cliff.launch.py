from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('k9_cliff_pkg')) / 'config' / 'cliff.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(package='k9_cliff_pkg', executable='cliff_detector',
             name='cliff_detector', output='screen',
             parameters=[config, {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])

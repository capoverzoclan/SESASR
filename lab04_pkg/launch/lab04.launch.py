from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (true for Gazebo/rosbag, false for real robot)',
    )

    ekf_node = Node(
        package='lab04_pkg',
        executable='ekf',
        name='ekf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ekf_extended_node = Node(
        package='lab04_pkg',
        executable='ekf_extended',
        name='ekf_extended',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bag_dir = os.path.expanduser('~/sesasr_course/lab_ros2_ws/src/SESASR/lab04_data')
    os.makedirs(bag_dir, exist_ok=True)

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/ground_truth',
            '/odom',
            '/ekf',
            '/ekf_ext',
            '/landmarks',
            '/imu',
        ],
        cwd=bag_dir,
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node,
        ekf_extended_node,
        rosbag_record,
    ])
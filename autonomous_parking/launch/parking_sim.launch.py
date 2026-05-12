import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_parking')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_dir, 'worlds', 'parking_lot.world')
    models_path = os.path.join(pkg_dir, 'models')

    # user_credential: 'general' (default) or 'handicapped'
    credential_arg = DeclareLaunchArgument(
        'user_credential',
        default_value='general',
        description='User parking credential: general | handicapped',
    )

    # Optional YOLOv8 model path (leave empty to use YAML ground truth)
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='',
        description='Path to YOLOv8 .pt model file (empty = yaml fallback)',
    )

    return LaunchDescription([
        credential_arg,
        yolo_model_arg,

        # Gazebo model & resource paths
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{models_path}",
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            f"{os.environ.get('GAZEBO_RESOURCE_PATH', '')}:{pkg_dir}",
        ),

        # Gazebo with parking lot world (ego_hatchback included in world)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # 1. Vision node — camera + YOLO (falls back to YAML if no model)
        Node(
            package='autonomous_parking',
            executable='vision_node.py',
            name='vision_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yolo_model': LaunchConfiguration('yolo_model'),
            }],
        ),

        # 2. Decision node — credential-based slot selection
        Node(
            package='autonomous_parking',
            executable='decision_node.py',
            name='decision_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'user_credential': LaunchConfiguration('user_credential'),
            }],
        ),

        # 3. Parking controller — observation points + parking execution
        Node(
            package='autonomous_parking',
            executable='parking_node.py',
            name='parking_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_parking')
    models_path = os.path.join(pkg_dir, 'models')
    world_file = os.path.join(pkg_dir, 'worlds', 'parking_lot.world')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{models_path}"
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            f"{os.environ.get('GAZEBO_RESOURCE_PATH', '')}:{pkg_dir}"
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),
    ])

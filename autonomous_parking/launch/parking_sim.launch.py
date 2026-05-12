import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Paths
    pkg_dir = get_package_share_directory('autonomous_parking')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(pkg_dir, 'worlds', 'parking_lot.world')
    models_path = os.path.join(pkg_dir, 'models')

    # Robot spawn position (entrance of parking lot - west gap)
    spawn_x = '-12.0'
    spawn_y = '0.0'

    return LaunchDescription([
        # 1. Gazebo 모델 경로 (hatchback, wheel_stopper 등 레포 모델)
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{models_path}:"
            f"{os.path.join(tb3_gazebo_dir, 'models')}"
        ),
        # 2. Gazebo 리소스 경로 (media/materials/... 텍스처 찾기)
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            f"{os.environ.get('GAZEBO_RESOURCE_PATH', '')}:{pkg_dir}"
        ),

        # 3. Gazebo + 주차장 월드
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # 4. Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # 5. TurtleBot3 스폰 (주차장 입구)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': spawn_x,
                'y_pose': spawn_y,
            }.items(),
        ),
    ])

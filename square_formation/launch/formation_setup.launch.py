#PDCP Multi robot systems
# launch file to send all the lines in the next comment section
# + use this line in the terminal: "ros2 launch square_formation formation_setup.launch.py"
# Last update: 15/oct/2025

"""#Create Gazebo world
ros2 launch multi_turtlebot_sim standalone_world.launch.py

#Define turtlebot model as burger
export TURTLEBOT3_MODEL=burger

#launch tortlebots
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=aire1 x_pose:=0.5 y_pose:=0.5
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=agua2 x_pose:=-0.5 y_pose:=0.5
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=tierra3 x_pose:=-0.5 y_pose:=-0.5
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=fuego4 x_pose:=0.5 y_pose:=-0.5

#connect odoms
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 aire1/odom agua2/odom
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 aire1/odom tierra3/odom
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 aire1/odom fuego4/odom"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Environment variable for TurtleBot3 model ---
    set_tb3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    # --- Launch Gazebo world ---
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('multi_turtlebot_sim'),
                'launch',
                'standalone_world.launch.py'
            )
        )
    )

    # --- Spawn four turtlebots with different prefixes and poses ---
    spawn_launch_path = os.path.join(
        get_package_share_directory('multi_turtlebot_sim'),
        'launch',
        'spawn_turtlebot3.launch.py'
    )

    robots = [
        {'name': 'aire1',   'x': '0.5',  'y': '0.5'},
        {'name': 'agua2',   'x': '-0.5', 'y': '0.5'},
        {'name': 'tierra3', 'x': '-0.5', 'y': '-0.5'},
        {'name': 'fuego4',  'x': '0.5',  'y': '-0.5'},
    ]

    spawn_robots = []
    for robot in robots:
        spawn_robots.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch_path),
                launch_arguments={
                    'robot_prefix': robot['name'],
                    'x_pose': robot['x'],
                    'y_pose': robot['y']
                }.items()
            )
        )

    # --- Static TF publishers to connect odoms ---
    tf_publishers = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_aire_agua',
            arguments=['0', '0', '0', '0', '0', '0', 'aire1/odom', 'agua2/odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_aire_tierra',
            arguments=['0', '0', '0', '0', '0', '0', 'aire1/odom', 'tierra3/odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_aire_fuego',
            arguments=['0', '0', '0', '0', '0', '0', 'aire1/odom', 'fuego4/odom']
        ),
    ]

    # --- Targets node (your multi-robot node) ---
    targets_node = Node(
        package='square_formation',
        executable='targets',
        name='multi_targets',
        output='screen',
        parameters=[{
            'namespaces': ['aire1', 'agua2', 'tierra3', 'fuego4']
        }]
    )

    # --- Combine everything ---
    return LaunchDescription([
        set_tb3_model,
        world_launch,
        *spawn_robots,
        *tf_publishers,
        targets_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = FindPackageShare('description')


    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_path, 'urdf', 'robot.xacro'])
        ]),
        value_type=str
    )

    # Gazebo world with physics tuned for balancing
    world_path = PathJoinSubstitution([pkg_path, 'worlds', 'balance.world'])

    udp_network_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('udp_network'),
                'launch',
                'udp_bridge.launch.py'
            ])
        )
    )

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', '-r', world_path],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'self_balance_robot',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.3'
            ],
            output='screen'
        ),

        # ROS-Gazebo Bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
                "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            ],
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_path, 'rviz', 'self_balance.rviz'])],
            output='screen'
        ),

        udp_network_launch
    ])
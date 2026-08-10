import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get share directory of go2_description
    go2_desc_share = get_package_share_directory('go2_description')
    
    # Process the xacro file dynamically at launch time
    import xacro
    xacro_path = os.path.join(go2_desc_share, 'urdf', 'go2_description.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    # Spawner for robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'publish_frequency': 50.0,
            'use_tf_static': True,
            'robot_description': robot_description,
        }],
    )

    # Spawn entity node
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'go2_robot', '-allow_renaming', 'true', '-z', '0.6', '-world', 'empty'],
    )

    # Controller spawners
    joint_states_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_states_controller", "--controller-manager", "/controller_manager"],
    )

    joint_group_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_effort_controller", "--controller-manager", "/controller_manager"],
    )

    # Simple Joint Controller Bridge Node
    simple_joint_controller = Node(
        package="go2_simple_joint_control",
        executable="simple_joint_controller",
        name="simple_joint_controller_bridge",
        parameters=[{
            'start_passive': LaunchConfiguration('start_passive')
        }],
        output="screen"
    )

    # Declare gz_args launch argument
    gz_args_arg = DeclareLaunchArgument(
        'gz_args',
        default_value='-r -v 4 empty.sdf',
        description='Arguments for Gazebo Sim'
    )

    start_passive_arg = DeclareLaunchArgument(
        'start_passive',
        default_value='true',
        description='Whether to start the robot in passive limp state'
    )

    return LaunchDescription([
        gz_args_arg,
        start_passive_arg,
        # Start Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', LaunchConfiguration('gz_args'))]),
            
        # Start GZ Parameter Bridge for Clock and IMU
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ],
            remappings=[
                ('/imu/data', '/trunk_imu')
            ],
            output='screen'
        ),

        robot_state_publisher,
        gz_spawn_entity,
        simple_joint_controller,

        # Spawners sequential event handlers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_states_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_states_controller,
                on_exit=[joint_group_effort_controller],
            )
        ),
    ])

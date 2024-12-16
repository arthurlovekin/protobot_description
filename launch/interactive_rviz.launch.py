from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# Launches RVIZ with interactive controls to simulate /joint_states
def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        
    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare('protobot_description'),
        'rviz', 
        'protobot_core.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output="screen",
        name='rviz_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_dir])
    
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('protobot_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_gazebo': 'true',
            'prefix': ""
            }.items()
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
        robot_state_publisher_launch,
        joint_state_publisher_gui_node
    ])

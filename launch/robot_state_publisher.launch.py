from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions.node import Node
from launch_ros.parameter_descriptions import ParameterValue

# Publish the tf tree (/tf) and /robot_description topic of the protobot based on the URDF
def generate_launch_description():
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='true')
    use_gazebo_arg = DeclareLaunchArgument('use_gazebo', default_value='false')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='""')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')
    prefix = LaunchConfiguration('prefix', default='""')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get URDF via xacro command
    robot_description_str = ParameterValue(
        Command([
            PathJoinSubstitution([
                FindExecutable(name="xacro")
            ]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("protobot_description"), "urdf", 
                "protobot.xacro"
            ]),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_gazebo:=",
            use_gazebo
        ]),
        value_type=str
    )

    # Publish the /robot_description and /tf topics
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_description_str, 
            }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_ros2_control_arg,
        use_gazebo_arg,
        prefix_arg,
        robot_state_publisher_node
    ])

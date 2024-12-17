from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions.node import Node
from launch_ros.parameter_descriptions import ParameterValue

# Compile Xacro files into a URDF, and publish this string on the /robot_description topic. Also publish the tf tree (/tf).
def generate_launch_description():

    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo', 
        default_value='false',
        description='Add gazebo-related tags to URDF if true'
    )
    prefix_arg = DeclareLaunchArgument(
        'prefix', 
        default_value='""',
        description='Prepend a prefix to every robot link and joint in the URDF'
    )

    use_gazebo = LaunchConfiguration('use_gazebo', default='false')
    prefix = LaunchConfiguration('prefix', default='""')

    # Get URDF via xacro command
    robot_description_str = ParameterValue(
        Command([
            PathJoinSubstitution([
                FindExecutable(name="xacro")
            ]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("protobot_description"), "urdf", 
                "protobot.urdf.xacro"
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
            'robot_description': robot_description_str, 
            }],
        output='screen'
    )

    return LaunchDescription([
        use_gazebo_arg,
        prefix_arg,
        robot_state_publisher_node
    ])

Overview
---
This package is intended to completely describe the physical characteristics of 
the protobot. On its own, the package contains xacro files and meshes that can 
be visualized by RVIZ. This package also contains additional files to enable 
functionality with ROS2 Control and the Gazebo simulator.

How to run
---
From your ros2 workspace (that contains protobot_description in the src 
directory), run
```
colcon build --packages-select protobot_description --symlink-install
source install/setup.bash
ros2 launch protobot_description interactive_rviz.launch.py
```

Package structure
---
- assets
    - pictures for the README
- launch
    - robot_state_publisher.launch.py will parse the xacro files into one urdf file, and then publish it as a string on /robot_description (which can then be used by gazebo and/or ros2 control). You can run `ros2 param get /robot_state_publisher_node robot_description` to see the full string, or test the urdf generation alone with `xacro <model_name>.xacro > <model_name>.urdf`
    - rviz.launch.py runs robot_state_publisher.launch.py and rviz. However, it requires something else (like gazebo or joint_state_publisher_gui) to publish joint states in order to succeed.
    - interactive_rviz.launch.py runs rviz.launch.py along with the joint_state_publisher_gui so that you can visualize the robot and move it around.
- meshes
    - stl files referenced by the xacro
- rviz
    - RViz config files
- urdf
    - urdf/xacro files that get compiled to one urdf string and then published
    by the robot_state_publisher package

Note that if you `colcon build` without `--symlink-install`, all of these files get copied into the install directory `protobot_ws/install/protobot_description/share/protobot_description`, which is the location that gets used by other packages and the rest of the robot. That means that when you make changes to the source files, they won't be seen until you rebuild. If you use `--symlink-install` then the files in the install directory will just be symlinked and you don't have to rebuild when you make changes. 

Topic structure
---
- /robot_description contains the urdf file as a raw string
    - Publisher: robot_state_publisher
    - Subscriber: joint_state_publisher_gui uses this to know which joint_states it should publish. Gazebo and ROS2 control also use this.
- /joint_states contains the current positions of the joints
    - Publisher: joint_state_publisher_gui (or a different package that actually reads the joint values)
    - Subscriber: robot_state_publisher uses this to publish /tf and /tf_static
- /tf and /tf_static contain the transform tree of the robot
    - Publisher: robot_state_publisher

Visual overview from [Articulated Robotics](https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-urdf)
![Robot Description Overview](assets/articulated_robotics_robot-description-overview.png)

Tutorials/Documentation:
- [ROS2 Control DiffBOt](https://control.ros.org/jazzy/doc/ros2_control_demos/example_2/doc/userdoc.html#diffbot)
- [ROS2 Control Demos example_2 (DiffBot)](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2)
- [Articulated Robotics](https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf)
- [ROS2 Control Demos example_9 (Gazebo)](https://github.com/ros-controls/ros2_control_demos/tree/master/example_9)

Notes:
- Generate urdf from xacro: `xacro <model_name>.xacro > <model_name>.urdf`
- Convert urdf to pdf graph: `urdf_to_graphviz ./protobot.urdf protobot`
- Colons in XML comments in the URDF can break the parsing
- convert urdf to sdf: `gz sdf -p protobot.urdf > protobot.sdf`

I was having trouble getting Gazebo to find the meshes in protobot_core.xacro
(dynamically setting GZ_SIM_RESOURCE_PATH to find the protobot_description 
package was hard). To fix this, I changed the syntax to include meshes from 
`<mesh filename="package://protobot_description/meshes/collision_wheel_v4.stl"/>`
to 
`<mesh filename="file://$(find protobot_description)/meshes/collision_wheel_v4.stl"/>`
(this resolves to:
`<mesh filename="/home/arthur/Projects/protobot_ws/install/protobot_description/share/protobot_description/meshes/collision_wheel_v4.stl"/>`)
Other options would be to add a .dsv.in hook to the protobot_description package 
that adds the meshes to the GZ_SIM_RESOURCE_PATH, or figure out a way to set 
the GZ_SIM_RESOURCE_PATH dynamically from the protobot_gazebo package.

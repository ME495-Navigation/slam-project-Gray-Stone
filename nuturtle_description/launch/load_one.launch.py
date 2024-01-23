from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.command import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.descriptions import ParameterValue
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions import EqualsSubstitution
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:

    return LaunchDescription([

        # Arguments
        DeclareLaunchArgument(name="use_jsp",
                              default_value="true",
                              description="select weather joint state publisher" +
                              " should be used, true or false. default true"),
        DeclareLaunchArgument(
            name="use_rviz",
            default_value="true",
            description="select weather to use rviz, true or false. default true"),
        DeclareLaunchArgument(name="color",
                              default_value="purple",
                              choices=["red", "green", "blue", "purple"],
                              description="select the color of the turtlebot."),

        # These are alternative ways of concatenate argument into strings
        # if not wanting to use the concatenation in Node's argument parameter.

        # SetLaunchConfiguration("rviz_file_name",
        #                        ["basic_", LaunchConfiguration("color"), ".rviz"]),
        # SetLaunchConfiguration("rviz_base_frame", [
        #     LaunchConfiguration("color"),
        #     "/base_frame",
        # ]),

        # Nodes to launch
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration("color"),
            parameters=[{
                'robot_description':
                    ParameterValue(  # Citation: ---------- [1] ----------
                        Command([
                            ExecutableInPackage("xacro", "xacro"), " ",
                            PathJoinSubstitution([
                                FindPackageShare('nuturtle_description'),
                                "urdf/turtlebot3_burger.urdf.xacro"
                            ]), " ", "color:=",
                            LaunchConfiguration("color")
                        ])),
                'frame_prefix': [LaunchConfiguration("color"), "/"]
            }]),
        Node(package='joint_state_publisher',
             executable="joint_state_publisher",
             namespace=LaunchConfiguration("color"),
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), 'true'))),
        Node(
            package='rviz2',
            namespace=LaunchConfiguration("color"),
            executable='rviz2',
            arguments=[["-f", LaunchConfiguration("color"), "/base_footprint"],
                       [
                           "-d",
                           PathJoinSubstitution(
                               [FindPackageShare('nuturtle_description'), "config/"]), "basic_",
                           LaunchConfiguration("color"), ".rviz"
                       ]],
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_rviz"), 'true')),
        ),
    ])

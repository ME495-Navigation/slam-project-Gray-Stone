from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.command import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.substitutions import ExecutableInPackage
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions import EqualsSubstitution
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:

    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument(
            name="use_jsp",
            default_value="true",
            description="select weather joint state publisher should be used, true or false. default true"),
        DeclareLaunchArgument(name="use_rviz",
                              default_value="true",
                              description="select weather to use rivz, true or false. default true"),


        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{
                 'robot_description':
                     Command([
                         ExecutableInPackage("xacro", "xacro"), " ",
                         PathJoinSubstitution(
                            #  [FindPackageShare('turtle_brick'), "turtle.urdf.xacro"]
                             "../urdf/turtlebot3_burger.urdf.xacro"
                             )
                     ])
             }]),


        # Node(package='joint_state_publisher_gui',
        #      executable="joint_state_publisher_gui",
        #      condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), 'gui'))),
        Node(package='joint_state_publisher',
             executable="joint_state_publisher",
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), 'true'))),
        Node(
            package='rviz2',
             executable='rviz2',
             arguments=[[
                 "-d",
                 "../config/basic_purple.rviz"
                #  PathJoinSubstitution(
                #      [FindPackageShare('turtle_brick'),LaunchConfiguration("rivz_config")]
                #     )
             ]],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_rviz"), 'true')),
             ),

             ])

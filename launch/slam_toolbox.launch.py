import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get file path 
    turtlecrab_param_file = os.path.join(get_package_share_directory('turtlecrab'), 'config', 'slam', 'slam_async.yaml')
    turtlecrab_map_root = os.path.join(get_package_share_directory('turtlecrab'), 'maps')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Declare simulation time, true for Gazebo, false for real robots
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description="Use Gazebo time")

    # Declare we want to resume map
    use_map_resume = LaunchConfiguration('use_map_file')
    use_map_resume_arg = DeclareLaunchArgument('use_map_file', default_value='False', description="Resume from existig map?")

    # Declare map file location load
    param_map_path = LaunchConfiguration('map_file')
    param_map_path_arg = DeclareLaunchArgument('map_file', default_value='', description="Map file path for continuation")
    
    # Hack No 1, in case you want to make the maps relative to the package and installable, default disabled
    param_map_path_full = [turtlecrab_map_root, '/', param_map_path]

    # Declare init pose
    param_map_pose = LaunchConfiguration('map_pose')
    param_map_pose_arg = DeclareLaunchArgument('map_pose', default_value="[0.0, 0.0, 0.0]", description="Map start pose")

    # Main SLAM node - conditional execution
    # Hack No 2, Nodes don't accept conditions, wrap the node around a Group that does accept conditionlas
    # Hack No 3, we need conditions to spawn the same node but with different parameters: default only sim time; resume: with sim time, map file and map pose
    slam_node_default = GroupAction([ 
        Node(
            parameters=[
                turtlecrab_param_file,
                {'use_sim_time': use_sim_time},
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen')
        ],
        condition=IfCondition(PythonExpression(['not ', use_map_resume])),
        scoped=False)

    # If resume, you need the posegraph/slam data file instead of the simple yaml/pgm file, program crashes if you feed the wrong data
    # Export: ros2 service call /serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : 'give_any_name'}"
    # Import: ros2 launch turtlecrab slam_toolbox.launch.py use_map_file:='True' map_file:='/home/marian/test' map_pose:="[-1.98661824, -0.49998818, 0.115]"
    # Pose is relative to what odom says, the example one is for turtlebot3 gazebo start
    # suffering
    slam_node_resume = GroupAction([
        Node(
            parameters=[
                turtlecrab_param_file,
                {'use_sim_time': use_sim_time},
                {'map_file_name': param_map_path},
                {'map_start_pose': param_map_pose}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen')
        ],
        condition=IfCondition(use_map_resume),
        scoped=False)

    #Optional load: allow basic navigation along side the mapping 
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py'))
    )

    ld = LaunchDescription()
    ld.add_action(use_map_resume_arg)
    ld.add_action(param_map_pose_arg)
    ld.add_action(param_map_path_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_node_default)
    ld.add_action(slam_node_resume)
    ld.add_action(navigation2_launch)
    
    return ld
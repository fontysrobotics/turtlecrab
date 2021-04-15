import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch and config directory
    turtlecrab_launch_dir = os.path.join(get_package_share_directory('turtlecrab'), 'launch')
    turtlecrab_nav_params = os.path.join(get_package_share_directory('turtlecrab'), 'config', 'nav')
    turtlecrab_slam_params = os.path.join(get_package_share_directory('turtlecrab'), 'config', 'slam', 'slam_localization.yaml')
    nav2_bt_trees = os.path.join(get_package_share_directory("nav2_bt_navigator"), "behavior_trees")

    # Specifi lifecycle nodes, order is important
    lifecycle_managed_nodes = ['controller_server','planner_server','recoveries_server', 'bt_navigator','waypoint_follower']

    # Declare we expect a map parameter
    map_slam_file = LaunchConfiguration('map')
    map_slam_file_arg = DeclareLaunchArgument('map', description='Absolute path to the slam-toolbox map files')
    # Declare we expect a map initial position parameter
    map_slam_pose = LaunchConfiguration('map_pose')
    map_slam_pose_arg = DeclareLaunchArgument('map_pose', description='Pose array for localisation start')
    # Declare we use simulation time, true for rosbag and gazebo
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True', description="Use Simulation clock?")

    # Create slam-toolbox localization node
    slam_node = Node(
        parameters=[
            turtlecrab_slam_params,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_slam_file},
            {'map_start_pose': map_slam_pose}
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    #Create group for navigation nodes:
    nav_group = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[
                turtlecrab_nav_params + "/controller_dwb.yaml",
                {"use_sim_time": use_sim_time}]),
        
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                turtlecrab_nav_params + "/planner.yaml",
                {"use_sim_time": use_sim_time}]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[
                turtlecrab_nav_params + "/recoveries.yaml",
                {"use_sim_time": use_sim_time}]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                turtlecrab_nav_params + "/bt_navigator.yaml",
                {"use_sim_time": use_sim_time},
                {"default_bt_xml_filename" : nav2_bt_trees + "/navigate_w_replanning_and_recovery.xml"}
            ]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[
                turtlecrab_nav_params + "/waypoint.yaml",
                {"use_sim_time": use_sim_time}])  
    ])

    # Create lifecycle manager node 
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_managed_nodes}])

    ld = LaunchDescription()
    ld.add_action(map_slam_file_arg)
    ld.add_action(map_slam_pose_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_node)
    ld.add_action(nav_group)
    ld.add_action(lifecycle_manager)

    return ld
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
    turtlecrab_config_dir = os.path.join(get_package_share_directory('turtlecrab'), 'config')
    nav2_bt_trees = os.path.join(get_package_share_directory("nav2_bt_navigator"), "behavior_trees")

    # Create pointer to the configuration file
    params_file_root = os.path.join(turtlecrab_config_dir, 'nav')
    # Specifi which nodes are managed
    lifecycle_managed_nods = ['map_server', 'amcl', 'controller_server','planner_server','recoveries_server', 'bt_navigator','waypoint_follower']

    # Declare we expect a map parameter
    map_yaml_file = LaunchConfiguration('map')
    map_yaml_file_arg = DeclareLaunchArgument('map', description='Absolute path to the map yaml file.')
    # Declare we use simulation time, true for rosbag and gazebo
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True', description="Use Simulation clock?")    

    #Create group for navigation nodes:
    nav_group = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[
                params_file_root + "/controller_dwb.yaml",
                {"use_sim_time": use_sim_time}]),
        
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                params_file_root + "/planner.yaml",
                {"use_sim_time": use_sim_time}]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[
                params_file_root + "/recoveries.yaml",
                {"use_sim_time": use_sim_time}]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file_root + "/bt_navigator.yaml",
                {"use_sim_time": use_sim_time},
                {"default_bt_xml_filename" : nav2_bt_trees + "/navigate_w_replanning_and_recovery.xml"}
            ]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[
                params_file_root + "/waypoint.yaml",
                {"use_sim_time": use_sim_time}])  
    ])

    # Create group for localization nodes
    local_group = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {"use_sim_time": use_sim_time},
                {"yaml_filename": map_yaml_file}]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file_root + "/amcl.yaml",
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
            {'node_names': lifecycle_managed_nods}])

    ld = LaunchDescription()
    ld.add_action(map_yaml_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(nav_group)
    ld.add_action(local_group)
    ld.add_action(lifecycle_manager)
    
    return ld
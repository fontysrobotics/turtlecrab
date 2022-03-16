
# What is turtlecrab?

Turtlecrab is a compilation of launch files and config files for launching Slam based navigation for mobile robots like the Turtlebot 3, suii or dee-dee in either simulation or in real life for **ROS2 Foxy**. 

## Using the Turtlebot 3 for Navigation
This package is intended to be used for launching either, the slam_toolbox or the nav_core with the right settings for the turtlebot3.

For launching the core navigation (**assumming that you already have a map**) type in the terminal:

`ros2 launch turtlecrab nav_core.launch.py map:='<map_location.yaml>'`

below is an example of how it should look (**Hint:** for getting the path, go to the folder in the UI, **not** the terminal, and press: `Ctrl + L`)

`ros2 launch turtlecrab nav_core.launch.py map:='/home/noxr/dev_ws/src/nav2_action_py_example/map/example_maze_03.yaml'`

## Using slam_toolbox for mapping big spaces
Yeah, it is anoying to map big spaces while driving the robot, so, using slam_toolbox 

For launching the slam_toolbox to do localization and mapping, then start the slam_toolbox:

`ros2 launch turtlecrab slam_toolbox.launch.py`

Then, your map data type must be `posegraph/slam` instead of the simple `yaml/pgm` file type. Then if you would like to save the map **for continuing mapping later**, type:

 - **For saving the map:** 
     - `ros2 service call /serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : 'give_any_name'}"`

 - **For opening the map to continue mapping:** 
     - `ros2 launch turtlecrab slam_toolbox.launch.py use_map_file:='True' map_file:='<path_to_the_map_location>' map_pose:="[<x, y, z>]"`
     - Where map_pose is relative to what odom says, the example one is for turtlebot3 gazebo start, also be aware that it is setup for simulation by default.

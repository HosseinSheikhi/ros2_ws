# ros2_ws

*NOTE: if someone wants to run the packages through IDE (have tested in clion and pycharm)
has to open IDEs from a terminal heading in dev_ws and setub is sourced. Press the debug button not run!*
- cd dev_ws/
- source install/setup.bash
- clion.sh


## How to run the whole project without launch file
#### Run turtlebot simulator in Gazebo
- cd turtlebot3_ws/
- source install/setup.bash
- ros2 launch turtlebot3_gazebo turtlebot3_factory.launch.py 

#### Run [overhead camera service package] (overhead_camera_service/README.md)
- cd dev_ws/
- source install/setup.bash
- ros2 run overhead_camera_service service_main

To check if it is running:
- ros2 service list
you must be able to find following service:
- /autonomous_robot/overhead_camera_service

#### Run [image segmentation package] (image_segmentation/README.md)
- cd dev_ws/
- source install/setup.bash
- ros2 run image_segmentation service_v2

#### Run [global planner packege] (global_planner/README.md)
- cd dev_ws/
- source install/setup.bash
- ros2 run global_planner global_planner_main


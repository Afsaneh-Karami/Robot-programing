# Object detection and counting for the agricultural robot:
The robot navigates by a topological map, avoids obstacles, and finds its path by move-based package. It detects the grapes by colour-filtering method. The 3D position of bunches of grapes, area limitation, and frame restriction were used to prevent double counting.<br><br
commands lines to run the solution: <br>
1. create a catkin workspace and put the my_package in src folder
2. roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small_s4_coarse launch_move_base:=false <br>
3. roslaunch my_package move_base.launch <br>
4. roslaunch my_package topo_nav.launch <br>
5. run two python codes in src folder:left_camera.py and right_camer <br>
Note: for launch files apply catkin_make and devel/setup.bash command in top level of catkin workspace


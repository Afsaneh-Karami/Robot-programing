# Grape detection and counting using image processing:<br />

In this project, the robot moves around the rows in the vineyard and takes photos with two cameras (left and right). The images were processed to find the number of grapes bunches while avoiding double counting, which was the most challenging part of this project. So, it needs to find the reasons for double counting and brings
solutions for them. The estimation showed that the double counting was about 20-23 perenct when the robot passed through a row twice. Some of it was related to the grapes
divided by leaves and counted as two bunches. A contour’s area limitation was applied to solve it, but it was impossible to increase it so much. It is related to the distance between the camera and the grapes, and a high value may result in missing some grapes if the distance increases. Another significant aspect of an agricultural robot is its navigation process. The robot should find its path to the goal while avoiding obstacles.<br />
<img width="303" alt="1" src="https://user-images.githubusercontent.com/78735911/217668371-afdc851b-04f8-43a2-90f5-5eae37895d87.png"> <br />
## Navigation section:<br />
For navigation, the robot needs to know the path and motion to avoid obstacles and a localization method to know its initial and goal position. There are two ways for navigation map-based uses a map of the environment, and reactive uses sensors to get local information about the environment surrounding the robot. In this project, a move-based approach was responsible for using global and local cost maps and implementing path planners to reach the goal. In map-based navigation, the map gives global knowledge about the static obstacle in advance, and sensors produce local information. Cost and topological maps depend on the environment, so they must change when-
ever the environment changes. In real-world cases, in some situations, a 3D cost map may be required. <br/>
In this project, the fake-localisation package was used to provide perfect localisation. In the real world, the robot can find its initial position by Adaptive Monte-Carlo Localisation (AMCL) algorithm. It uses sample locations in the map and current scan information matching each sample. After a while, it can find the correct position and the map fitted by the environment correctly.
* creating a topological map <br/>
A topological map is mainly used for complex and large
environments. The robot must react differently in each zone,
so the path-planning method needs a graph-like map. In this
project, a topological map was chosen because it is more
convenient to move the robot in a straight line and at a
constant distance regarding each row. For example, the robot
goes through a row, then turns to the other side of the row
and finally returns to its initial position. The goal indicates by
clicking on each point, and the robot moves toward it while
following the edge and its direction (follow the edge as far
as there is no dynamic obstacle). I use a topological map that
matches all words.
* move-based <br/>
* Move-base package is responsible for moving the robot
to the destination based on two motion planners: the global
and local path planners (each planner uses its own cost or
occupancy map). The global one uses a search algorithm to
find a suitable path free of static obstacles to the goal. The
local path planner executes the planned trajectory while avoiding dynamic obstacles. The local path planner produces the
velocity command, and its modified parameters are max and
min velocity and acceleration. It has two built-in approaches,
dynamic window approach (DWA) and Trajectory Rollout.
In both algorithms, three component contributes to producing
suitable velocity, which is the target heading (distance to the
goal), clearance (avoiding obstacle), and velocity (supports fast
movements) [1]. A DWA simulates one step of movement
to score the function, and trajectory rollout goes for more
steps. So, in the trajectory rollout, paths that lead to the
obstacle are ignored. In fact, in the scoring function, more
steps make the scale of avoiding obstacles higher compared
to the goal distance scale. Based on the narrow rows in this
project, DWA was considered to decrease obstacle avoidance
sensitivity and go faster toward the destination. There are three
built-in functions carrot-planner, navfn (Dijkstra AL), and
global-planner(A* AL) for global path planner. A* algorithm
considers both the cost map and distance to the goal. The
Dijkstra considers just cost value. In comparison, A* pays
more attention to the path leading to the goal, and Dijkstra can
better avoid several obstacles. In this project, both algorithms
were tested, and Dijkstra was selected because it goes to
discover the cost value for a wider space. It could better find
its path when there were several adjacent obstacles, although
its computation time is higher. So, the probability of getting
stuck is lower. Two recovery behaviour, conservative-reset and
aggressive-reset, were activated in the planner yaml file as a
solution when the robot gets stuck and put the value of 1.1
for conservative-reset. This value should be near the max of
inflation-radius*cost-scaling-factor of local. Therefore it can
be activated before going so close to obstacles and getting
stuck with the cost map. The robot should have the ability to
rotate for recovery behaviour.
To be able to change the parameter of the move-base package,
this package was run from the local folder. Two important
parameters that indicate the value of the cost map are inflationradius and cost-scaling-factor, which are related to obstacle
avoidance. In the simulation environment, the rows that the
robot can cross are narrow, and, in some worlds, the trees
grew a lot. Therefore, these parameters must be low to allow
the robot to move through this environment. Even if the
parameters were decreased to the lowest value, the robot can
hardly move in the world vineyard stages 3 and 4 and mostly
get stuck. The value of the inflation-radius and cost-scalingfactor for local are 0.67 and 1.5, respectively. The values for
the global cost map are 0.4 and 2 for the inflation-radius and
cost-scaling-factor, respectively. The local parameter should be
higher than the global because the global cost map is on top
of the local cost map. Local parameters are more related to
the robot’s size and free space in the environment. The robot
should be able to move and turn. The obstacle-range is 1.5 m,
the maximum value that an obstacle can consider in the cost
map (1.5 m from the robot), and the raytrace-range is 5.5 m,
which is the distance that sensor can see the obstacles.

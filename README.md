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


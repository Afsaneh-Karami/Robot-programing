# Grape detection and counting using image processing:<br />

In this project, the robot moves around the rows in the vineyard and takes photos with two cameras (left and right). The images were processed to find the number of grapes bunches while avoiding double counting, which was the most challenging part of this project. So, it needs to find the reasons for double counting and brings
solutions for them. The estimation showed that the double counting was about 20-23 perenct when the robot passed through a row twice. Some of it was related to the grapes
divided by leaves and counted as two bunches. A contour’s area limitation was applied to solve it, but it was impossible to increase it so much. It is related to the distance between the camera and the grapes, and a high value may result in missing some grapes if the distance increases. Another significant aspect of an agricultural robot is its navigation process. The robot should find its path to the goal while avoiding obstacles.<br />
<img width="303" alt="1" src="https://user-images.githubusercontent.com/78735911/217668371-afdc851b-04f8-43a2-90f5-5eae37895d87.png"> <br />
## Navigation section:<br />
For navigation, the robot needs to know the path and motion to avoid obstacles and a localization method to know its initial and goal position. There are two ways for navigation map-based uses a map of the environment, and reactive uses sensors to get local information about the environment surrounding the robot. In this project, a move-based approach was responsible for using global and local cost maps and implementing path planners to reach the goal. In map-based navigation, the map gives global knowledge about the static obstacle in advance, and sensors produce local information. Cost and topological maps depend on the environment, so they must change when-
ever the environment changes. In real-world cases, in some situations, a 3D cost map may be required. <br/>
In this project, the fake-localisation package was used to provide perfect localisation. In the real world, the robot can find its initial position by Adaptive Monte-Carlo Localisation (AMCL) algorithm. It uses sample locations in the map and current scan information matching each sample. After a while, it can find the correct position and the map fitted by the environment correctly.
* creating a topological map (GOTO [map folder link](https://github.com/Afsaneh-Karami/my_package/blob/main/maps/foo3.tmap2)) <br />
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
matches all words.<br/>
![image](https://user-images.githubusercontent.com/78735911/217673959-c08d6583-713c-461a-b058-e3c46858277a.png)

* move-based (GOTO [config folder link](https://github.com/Afsaneh-Karami/my_package/tree/main/config)) <br />
Move-base package is responsible for moving the robot to the destination based on two motion planners: the global and local path planners (each planner uses its own cost or occupancy map). The global one uses a search algorithm to find a suitable path free of static obstacles to the goal. The local path planner executes the planned trajectory while avoiding dynamic obstacles. The local path planner produces the velocity command, and its modified parameters are max and
min velocity and acceleration. It has built-in approaches, dynamic window approach (DWA). In this algorithms, three component contributes to producing suitable velocity, which is the target heading (distance to the goal), clearance (avoiding obstacle), and velocity (supports fast movements). There are three built-in functions carrot-planner, navfn (Dijkstra AL), and global-planner(A* AL) for global path planner. A* algorithm considers both the cost map and distance to the goal. The Dijkstra considers just cost value. In comparison, A* pays more attention to the path leading to the goal, and Dijkstra can better avoid several obstacles. In this project, both algorithms were tested, and Dijkstra was selected because it goes to discover the cost value for a wider space. It could better find its path when there were several adjacent obstacles, although its computation time is higher. So, the probability of getting stuck is lower. Two recovery behaviour, conservative-reset and aggressive-reset, were activated in the planner yaml file as a solution when the robot gets stuck and put the value of 1.1 for conservative-reset. This value should be near the max of inflation-radius*cost-scaling-factor of local. Therefore it can be activated before going so close to obstacles and getting stuck with the cost map. The robot should have the ability to
rotate for recovery behaviour.
To be able to change the parameter of the move-base package,
this package was run from the local folder. Two important
parameters that indicate the value of the cost map are inflation-radius and cost-scaling-factor, which are related to obstacle
avoidance. In the simulation environment, the rows that the
robot can cross are narrow, and, in some worlds, the trees
grew a lot. Therefore, these parameters must be low to allow
the robot to move through this environment. Even if the
parameters were decreased to the lowest value, the robot can
hardly move in the world vineyard stages 3 and 4 and mostly
get stuck. The value of the inflation-radius and cost-scaling-factor for local are 0.67 and 1.5, respectively. The values for
the global cost map are 0.4 and 2 for the inflation-radius and
cost-scaling-factor, respectively. The local parameter should be
higher than the global because the global cost map is on top
of the local cost map. Local parameters are more related to
the robot’s size and free space in the environment. The robot
should be able to move and turn. The obstacle-range is 1.5 m,
the maximum value that an obstacle can consider in the cost
map (1.5 m from the robot), and the raytrace-range is 5.5 m,
which is the distance that sensor can see the obstacles.
* image processing with OpenCV (GOTO [src folder link]((https://github.com/Afsaneh-Karami/my_package/tree/main/src))) <br />
1. detection of grapes’ bunches
2. The camera for RGB images has a field of view (FOV)
of 84.1 (horizontal) and 54 (vertical) degrees and an image
resolution of 1920x1080. A camera with a FOV of 70 (horizontal) and 61 (vertical) degrees and an image resolution of
512x424 was used for depth images. An online application on
the website was applied for FOV calculation [5]. The code
had five subscribers, one for RGB image information, one
for depth image, one for camera info, one for transforming
coordination from 2D to 3D, and the last one for transmitting 3D coordinates information between two cameras. Two
publishers publish the number of detected objects and their
3D coordinates. The second publisher is used to publish 3D
coordinates. So, whenever a new camera frame comes, a list
of the 3D coordinate of detected grapes is published. Before
processing new contours of the new frame (in each camera),
the subscriber gets information about previous frames. The
HSV image format was selected for colour filtering because it
is more robust toward external lighting changes. In real-world
cases, lightning and shadow change during the day. With minor
changes in external lighting, the Hue values change lesser than
RGB values, so it is a better detector of colours. So, HSV
can separate colour from lightness and gives the ability to do
different operations on the colour. It is possible to equip the
robot with light in real-world applications for steady lighting.
Several pictures from the camera were considered to find the
lower and upper range of HSV for the inRange function.
Whenever the save-pic variable in the code was true, and a new
frame came, the picture was saved for future processing. By
cropping the image, light and dark grapes were separated, and
their values were used for the inrange function. As the whole
bunch as an object was needed, the blur function was applied
before finding the contour. The best blur functions between
GaussianBlur, blur,medianBlur, and bilateral filter were blur by
the kernel size 8. It was determined by trial and error. In some
simulated worlds, grapes are green which needs to change the
parameters of the mask function. In real-world cases, more
masks are needed for different kinds and colours of grapes.<br/>
![image](https://user-images.githubusercontent.com/78735911/217674666-08cd406b-2baa-4f23-89cf-a6307ecb0f51.png)<br/>
2. avoid double counting <br/>
After creating contours and finding their properties, like
centre point, radius and 3D coordinate, they were processed to
avoid double counting. Several filters were applied, including
the contours’ area limitation, object location in 3D coordinates,
and frame restriction. Area limitation and tolerance used in
3D coordinates are parameters that should be chosen based
on the kind and size of grapes, so they are dependent on the
environment.
1- contours’ area limitation: Some leaves divide bunches into
two parts, so they were detected as two separate bunch even if
the 3D location filter was applied. Leaves cover some bunches,
and a small amount of them can be seen from the camera, so it
is better to ignore them, and they count when the robot goes to
another side of the row. If it is not ignored, it will count twice
from both sides. Limiting the contours’ area makes it possible
to avoid double counting in situations like these. A minimum
value of the contours’ area was considered. This parameter can
be learned based on the world and the camera’s distance to
the row. To find the best value, I put the area of each contour
on the images and, by visualisation, found a suitable value.
2- object location in 3D coordinates: Another solution to avoid
double counting is to save the bunch’s location regarding
the map frame coordinate and compare them with the new
bunch coordinate. If it is not added to the list before, it
is accepted as a new bunch and added one value to the
object counter. However, it should be considered that some
bunches are split by leaves and may not be eliminated by
the area filters. Depending on the camera angle in some
frames, a whole bunch may be seen (without leaves) and
counted. From another frame, split by the leaves and counted
as two objects. Each of these three objects has a different
3D coordinate and can not filter at this stage. In order to
avoid it, frame limitations and relative tolerance were added.
The function minEnclosingCircle was used to find the best
value for tolerance. This function creates a circle to evolve the
contour and gives the circle’s radius. I considered the radius
and the minimum distance between the nearest bunches centre
and chose the minimum value as an index to show the safe
area around each bunch. In this area, the possibility of the
existence of another bunch is low, so it is a good indicator for
tolerance in comparing processes.
In order to create a 3D coordinate of the object, the following
step is needed.
A.To find the centre point of the contour in an RGB image
using the momentum function
B. Project this point in the depth image and find the depth
value (the distance between the camera and detected bunch)
based on FOV and centre coordinate of the depth image
C. Appling the projectPixelTo3dRay function from Pinhole-CameraModel class to get the object’s 3D location in the
camera coordinate
D. Using the class PoseStamped to define the 3D position in
the camera frame
E. Using the transformPose function from the tf package to
transfer the 3D position from the camera to the map frame
In step B, as the FOV of the colour camera is higher than the
depth, the image of the depth camera can not cover all the
cells of the RGB image. So, there are some bunches in RGB
images that, when projected to depth images, are put outside
the maximum size of the depth image. I just put a value of
100 for depth in this situation, and in code, whenever the
depth becomes 100, it ignores this contour. Even though some
information is missing, there is no other choice. Choosing the
same FOV for both cameras is a good way. As there is some
gap between grapes in a bunch, in some situations, the centre
of the contour is a cell in this gap, so the depth value is blank,
and the X, Y, and Z value of the 3D coordinate is nan value.
The point’s position was changed a little, and a while loop
was used to ensure that the depth value was correct.
3- Frame restriction: As the counting process happens as the
robot moves and the robot’s speed can be varied, the pictures
of the two consequence frames overlap. Counting a grape in
two frames may not be addressed by the before steps. For
example, because of some obstacles, the robot does not follow
the topological map’s edge and moves nearer or farther to the
row, influencing the filtration power. Finding the X position
(regarding the map coordinate system) of the max and min
image’s cells at each frame (Xf1 and Xf2), and in the new
frame, the X coordinate of the detected bunch is compared by
this value. It is not counted if it belongs to the previous frame,
which depends on the movement direction.<br/>
![fig1](https://user-images.githubusercontent.com/78735911/217675669-6af64374-52d7-4235-8efb-c8f3a0a51af3.png)

In a real-world case, adding unique features to avoid double
counting, like shape, the number of grapes, and the average
colour in the contour pixel, improve the accuracy of counting. Also, the ground is not flat and increases the noise in
coordinate transforming. The slippage of wheels should be considered too.





# Grape detection and counting using image processing:

In this project, the robot moves around the rows in the vineyard and takes photos with two cameras (left and right). The images were processed to find the number of grapes bunches while avoiding double counting, which was the most challenging part of this project. So, it needs to find the reasons for double counting and brings
solutions for them. The estimation showed that the double counting was about 20-23 perenct when the robot passed through a row twice. Some of it was related to the grapes
divided by leaves and counted as two bunches. A contour’s area limitation was applied to solve it, but it was impossible to increase it so much. It is related to the distance between the camera and the grapes, and a high value may result in missing some grapes if the distance increases. Another significant aspect of an agricultural robot is its navigation process. The robot should find its path to the goal while avoiding obstacles.\\
<img width="303" alt="1" src="https://user-images.githubusercontent.com/78735911/217668371-afdc851b-04f8-43a2-90f5-5eae37895d87.png">


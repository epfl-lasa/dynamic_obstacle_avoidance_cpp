
# ObstacleAvoidance Algorithm
---
This package contains a dynamic obstacle avoidance algorithm for concave and convex obstacles as developped in [1]. The algorithm is based on the work of [2].
This module requires python 3 including certain libraries which can be found in requirements-pip.txt.
---

Clone code from github and make sure all pip requirement are met:

```
git clone 
pip install -r requirements-pip.txt
```
## Quick start
Several examples of the obstacle avoidance algorithm have been ipmlemented.
<p align="center">
<img src="https://raw.githubusercontent.com/epfl-lasa/dynamic_obstacle_avoidance_linear/master/blob/wheelchairObstacles.png"  width="350"></>
  <img src="https://raw.githubusercontent.com/epfl-lasa/dynamic_obstacle_avoidance_linear/master/blob/wheelchairSimulation.png"  width="350"></>

### Vector fields
Different examples of the vector field simulation can be launched by running the script
```
examples_vectorField.py
```
The simulation number can be specified to run each specific simulation. The resolution indicates the number of grid points along each axis. Further more figures can be saved automatically into the <<fig>> folder.

Custom vector fields can be created using by calling the class
'''
Simulation_vectorFields() in lib_visalization/vectorField_visualization.py
'''

### Animated visualization
Different animated examples with static and non-static obstacles can be found in:
'''
examples_animation.py
'''
The simulation number can be specified to choose between the animations. Further it can be saved directly to a MP4 video.

Custom vector animation can be created by running the function
'''
run_animation() in lib_visalization/animated_simulation.py
'''


## Obstacle Class
For each obstacle of an ellipsoid form, a class instanse of "lib_obstacleAvoidance/obstacle_class.py" has to be defined. This desires several paramters such as center position x0, axis length a, surface curvature p, orientation th_r.
Moving obstacles additionally have a linear velocity xd and an angular velocity w.

For the modulation towards a general obstacle needs a reference point within the obstacle, the distance to the obstacle and the tangent hyperplane. 

## Modulation
An initial (linear) dynamical system is modulated around obstacles. The modulation works in real-time and dynamically around any number of obstacles. Convergence towards an attractor can be ensured, as long as intersecting obstacles can be described with a star shape.
The main modulation is happening in the file: "lib_obstacleAvoidance/linear_modulations.py" 
It contains the main functions 

Any obstacle of this form can be directly used with the "lib_obstacleAvoidance/lib_modulation.py" library. Beware, currently there exists sever modulation function, where the most recent one is "obs_modulation_interpolation_velocity()".
Furhtemore, there exist the choice of RK4 integration to evaluate the velocity. (Currently the expected movement is not included in this higher order integration.)

### Reference Point
At the heart of the present obstacle avoidance algorithm lies the correct placement of the reference point, which defines the splitting of the DS.
For good placment to functions have to be applied to the list of obstacles to correctly place the reference point:
- obs_common_section.py - numerically checks if obstacles have a common section, and if there exists one, the reference point is placed within it.
- obs_dynamic_center(_3d).py - if no common center exists with an other obstacle, the reference point is displaced from the center of the obstacle with respect to the distances to the other obstacles. This allows a smooth transition between the reference point at the center of the obstacle and sharing the reference point with another obstacle. 

### Concave obstacles
Complexer obstacles can either be formed using several ellipses, which already allows to form many star shaped obstacles.
Another approach can be, to form more complex obstacles with an analytical description of the obstacles.

## Simulation
Two form of visualization exist:

### Vector fields (Streamlines) - 2D
The flow of the obstacle is simulated as streamlines. It used the function defined in "simulationVectorFields.py". Examples of this can be found in
examples_vectorField.py

### Dynammic Simulation 2D / 3D
The dynamic simulation for implemented for two and three dimensionsional obstacles can be found in
examples_dynamic.py

Beware, the 3D simulation tends to show problems concerning the visualization of the obstacles and will be updated in the near future. 

## Structure


## Obstacle Avoidance Tools
General tools, susch are found in "lib_obstacleAvoidance/lib_obstacleAvoidance.py" it includes:
- Calculating rotation matrix
- Caluclate weight, based on input distance array.
- check_collision > checks if a point lies within any of the obstacles


**References**     
> [1] Huber, Lukas, Aude Billard, and Jean-Jacques E. Slotine. "Avoidance of Convex and Concave Obstacles with Convergence ensured through Contraction." IEEE Robotics and Automation Letters (2019).
> [2] Khansari-Zadeh, Seyed Mohammad, and Aude Billard. "A dynamical system approach to realtime obstacle avoidance." Autonomous Robots 32.4 (2012): 433-454.



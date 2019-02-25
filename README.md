
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

## Obstacle Class
For each obstacle of an ellipsoid form, a class instanse of "lib_obstacleAvoidance/obstacle_class.py" has to be defined. This desires several paramters such as center position x0, axis length a, surface curvature p, orientation th_r.
Moving obstacles additionally have a linear velocity xd and an angular velocity w.

For the modulation towards a general obstacle needs a reference point within the obstacle, the distance to the obstacle and the tangent hyperplane. 

## Modulation
An initial (linear) dynamical system Any obstacle of this form can be directly used with the "lib_obstacleAvoidance/lib_modulation.py" library. Beware, currently there exists sever modulation function, where the most recent one is "obs_modulation_interpolation_velocity()".
Furhtemore, there exist the choice of RK4 integration to evaluate the velocity. (Currently the expected movement is not included in this higher order integration.)
The modulation is of the form
$$
\dot \xi = M (\xi) f (\xi)
$$

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



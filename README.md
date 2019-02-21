# ObstacleAvoidance Algorithm
Implemented in Python

## Obstacle Class
For each obstacle of an ellipsoid form, a class instanse of "lib_obstacleAvoidance/obstacle_class.py" has to be defined. Several variables as the state and shape of the obstacle must be defined.

An obstacle of this form can be directly used with the "lib_obstacleAvoidance/lib_modulation.py" library. Beware, currently there exists sever modulation function, where the most recent one is "obs_modulation_interpolation_velocity()".
Furhtemore, there exist the choice of RK4 integration to evaluate the velocity. (Currently the expected movement is not included in this higher order integration.)

### Complexer Obstacle
Complexer obstacles can either be formed using several ellipses, which already allows to form many star shaped obstacles.
Or an example how to treat convex obstacles can also be found in "analysisMetric.py", for a cross-shaped obstacle.


## Simulation
Two form of visualization exist:

### Vector fields (Streamlines) - 2D
The flow of the obstacle is simulated as streamlines. It used the function defined in "simulationVectorFields.py". Examples of this can be found in
- vectorFeidl_examples.py
- multipleObstacles_vectorField.py
- figuresReport.py

### Dynammic Simulation 2D / 3D
The dynamic simulation for implemented for two and three dimensionsional obstacles can be found in
dynamicSimulation.py

Beware, the 3D simulation tends to show problems concerning the visualization of the obstacles.

## Obstacle Avoidance Tools
General tools, susch are found in "lib_obstacleAvoidance/lib_obstacleAvoidance.py" it includes:
- Calculating rotation matrix
- Caluclate weight, based on input distance array.
- check_collision > checks if a point lies within any of the obstacles

### Reference Point
At the heart of the present obstacle avoidance algorithm lies the correct placement of the reference point, which defines the splitting of the DS.
For good placment to functions have to be applied to the list of obstacles to correctly place the reference point:
- obs_common_section.py - numerically checks if obstacles have a common section, and if there exists one, the reference point is placed within it.
- obs_dynamic_center(_3d).py - if no common center exists with an other obstacle, the reference point is displaced from the center of the obstacle with respect to the distances to the other obstacles. This allows a smooth transition between the reference point at the center of the obstacle and sharing the reference point with another obstacle. 


## Analytical Analsis
This folder contains several files for the analytical analysis of contraction/divergence of the system to proof stabilty. 

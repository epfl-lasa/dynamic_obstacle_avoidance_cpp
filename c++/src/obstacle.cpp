#include "DynamicObstacleAvoidance/obstacle.hpp"

Obstacle::Obstacle(VectorXf& position, VectorXf& orientation, VectorXf& axis, VectorXf& curvature, double safety_margin): 
position(position), orientation(orientation), axis(axis), curvature(curvature), safety_margin(safety_margin)
{}

Obstacle::~Obstacle() {}
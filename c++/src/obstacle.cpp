#include "DynamicObstacleAvoidance/obstacle.hpp"

Obstacle::Obstacle(Eigen::VectorXf& center_position, Eigen::VectorXf& center_orientation, Eigen::VectorXf& reference_position, double safety_margin): 
center_position(center_position), center_orientation(center_orientation), reference_position(reference_position), safety_margin(safety_margin)
{}

Obstacle::Obstacle(Eigen::VectorXf& center_position, Eigen::VectorXf& center_orientation, double safety_margin):
Obstacle(center_position, center_orientation, center_position, safety_margin)
{}

Obstacle::~Obstacle() {}

Eigen::VectorXf Obstacle::compute_normal_to_external_point(const Eigen::VectorXf& external_point) const
{}

double Obstacle::compute_distance_to_external_point(const Eigen::VectorXf& external_point) const
{}
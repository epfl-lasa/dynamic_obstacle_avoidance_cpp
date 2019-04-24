#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"

Obstacle::Obstacle(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, Eigen::Vector3f& reference_position, double safety_margin): 
center_position(center_position), center_orientation(center_orientation), reference_position(reference_position), safety_margin(safety_margin)
{}

Obstacle::Obstacle(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, double safety_margin):
Obstacle(center_position, center_orientation, center_position, safety_margin)
{}

Obstacle::~Obstacle() {}

Eigen::Vector3f Obstacle::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{}

double Obstacle::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{}
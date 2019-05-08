#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"

Obstacle::Obstacle():state(Eigen::Vector3f(0,0,0))
{}

Obstacle::Obstacle(const State& state, const float& safety_margin):
state(state), reference_position(state.get_position()), safety_margin(safety_margin) 
{}

Obstacle::Obstacle(const State& state, const Eigen::Vector3f& reference_position, const float& safety_margin):
state(state), reference_position(reference_position), safety_margin(safety_margin)
{}

Obstacle::~Obstacle() 
{}

Eigen::Vector3f Obstacle::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{
	std::cerr << "Fonction of abstract class obstacle used" << std::endl;
}

float Obstacle::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{
	std::cerr << "Fonction of abstract class obstacle used" << std::endl;
}

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"

Obstacle::Obstacle():state(Eigen::Vector3d(0,0,0))
{}

Obstacle::Obstacle(const State& state, const double& safety_margin):
state(state), reference_position(state.get_position()), safety_margin(safety_margin) 
{}

Obstacle::Obstacle(const State& state, const Eigen::Vector3d& reference_position, const double& safety_margin):
state(state), reference_position(reference_position), safety_margin(safety_margin)
{}

Obstacle::Obstacle(const double& cx, const double& cy, const double& cz, const double& safety_margin):
state(cx, cy, cz), reference_position(cx, cy, cz), safety_margin(safety_margin) 
{}

Obstacle::~Obstacle() 
{}

Eigen::Vector3d Obstacle::compute_normal_to_agent(const Agent& agent) const
{
	std::cerr << "Fonction of abstract class obstacle used" << std::endl;
}

double Obstacle::compute_distance_to_agent(const Agent& agent) const
{
	std::cerr << "Fonction of abstract class obstacle used" << std::endl;
}

std::pair<bool, Eigen::Vector3d> Obstacle::find_intersection_center(const Obstacle& other_obstacle) const
{
	std::cerr << "Fonction of abstract class obstacle used" << std::endl;
}
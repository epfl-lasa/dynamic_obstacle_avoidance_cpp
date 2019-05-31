#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"

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
	std::cerr << "Fonction compute_normal_to_agent of abstract class obstacle used" << std::endl;
	return Eigen::Vector3d();
}

double Obstacle::compute_distance_to_agent(const Agent& agent) const
{
	std::cerr << "Fonction compute_distance_to_agent of abstract class obstacle used" << std::endl;
	return 0.0;
}

void Obstacle::draw() const
{
	std::cerr << "Fonction draw of abstract class obstacle used" << std::endl;
}

bool Obstacle::is_intersecting(const Obstacle& other_obstacle) const
{
	if(other_obstacle.get_type() == "Ellipsoid") 
	{
		return this->is_intersecting_ellipsoid(static_cast<const Ellipsoid&>(other_obstacle));
	}
	else if(other_obstacle.get_type() == "Aggregate")
	{
		return this->is_intersecting_aggregate(static_cast<const Aggregate&>(other_obstacle));
	}
	else
	{
		std::cerr << "Fonction is_intersecting not implemented for this type of obstacles" << std::endl;
	}	
	return false;
}

bool Obstacle::is_intersecting(const std::deque<std::unique_ptr<Obstacle> >& other_obstacles) const
{
	bool intersecting = false;
	for(auto& o:other_obstacles) intersecting = (intersecting || this->is_intersecting(*o));
	return intersecting;
}

bool Obstacle::is_intersecting_ellipsoid(const Ellipsoid& other_obstacle) const
{
	std::cerr << "Fonction is_intersecting with Ellipsoid of abstract class obstacle used" << std::endl;
	return false;
}

bool Obstacle::is_intersecting_aggregate(const Aggregate& other_obstacle) const
{
	bool intersecting = false;
	for(auto& p:other_obstacle.get_primitives()) intersecting = (intersecting || this->is_intersecting(*p));
	return intersecting;
}

std::unique_ptr<Obstacle> Obstacle::clone() const
{
	return std::unique_ptr<Obstacle>(this->implicit_clone());
}

Obstacle* Obstacle::implicit_clone() const
{
	std::cerr << "Fonction implicit_clone of abstract class obstacle used" << std::endl;
}

double Obstacle::get_repulsion_factor(const Agent& agent) const
{
	std::cerr << "Fonction get_repulsion_factor of abstract class obstacle used" << std::endl;
}
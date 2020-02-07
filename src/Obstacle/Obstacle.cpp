#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"

namespace DynamicObstacleAvoidance
{
	Obstacle::Obstacle(const std::string& name, double safety_margin):
	name(name), state(Eigen::Vector3d::Zero()), safety_margin(Eigen::Array3d(safety_margin, safety_margin, safety_margin))  
	{}

	Obstacle::Obstacle(const std::string& name, const Eigen::Array3d& safety_margin):
	name(name), state(Eigen::Vector3d::Zero()), safety_margin(safety_margin)
	{}

	Obstacle::Obstacle(const std::string& name, double cx, double cy, double cz, double safety_margin):
	name(name), state(cx, cy, cz), reference_position(cx, cy, cz), safety_margin(Eigen::Array3d(safety_margin, safety_margin, safety_margin)) 
	{}

	Obstacle::Obstacle(const std::string& name, double cx, double cy, double cz, const Eigen::Array3d& safety_margin):
	name(name), state(cx, cy, cz), reference_position(state.get_position()), safety_margin(safety_margin)
	{}

	Obstacle::Obstacle(const std::string& name, const State& state, double safety_margin):
	name(name), state(state), reference_position(state.get_position()), safety_margin(Eigen::Array3d(safety_margin, safety_margin, safety_margin))
	{}

	Obstacle::Obstacle(const std::string& name, const State& state, const Eigen::Array3d& safety_margin):
	name(name), state(state), reference_position(state.get_position()), safety_margin(safety_margin)
	{}

	Obstacle::Obstacle(const std::string& name, const State& state, const Eigen::Vector3d& reference_position, double safety_margin):
	name(name), state(state), reference_position(reference_position), safety_margin(Eigen::Array3d(safety_margin, safety_margin, safety_margin))
	{}

	Obstacle::Obstacle(const std::string& name, const State& state, const Eigen::Vector3d& reference_position, const Eigen::Array3d& safety_margin):
	name(name), state(state), reference_position(reference_position), safety_margin(safety_margin)
	{}

	Obstacle::~Obstacle() 
	{}

	Eigen::Vector3d Obstacle::compute_normal_to_agent(const Agent&) const
	{
		std::cerr << "Fonction compute_normal_to_agent of abstract class obstacle used" << std::endl;
		return Eigen::Vector3d();
	}

	double Obstacle::compute_distance_to_point(const Eigen::Vector3d&, const Eigen::Array3d&) const
	{
		std::cerr << "Fonction compute_distance_to_point of abstract class obstacle used" << std::endl;
		return 0.0;
	}

	double Obstacle::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{
		return this->compute_distance_to_point(point, Eigen::Array3d(safety_margin, safety_margin, safety_margin));
	}

	double Obstacle::compute_distance_to_agent(const Agent& agent) const
	{
		return this->compute_distance_to_point(agent.get_position(), agent.get_safety_margin());
	}

	void Obstacle::draw(const std::string&, bool) const
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

	bool Obstacle::is_intersecting(const std::deque<std::shared_ptr<Obstacle> >& other_obstacles) const
	{
		bool intersecting = false;
		for(auto& o:other_obstacles) intersecting = (intersecting || this->is_intersecting(*o));
		return intersecting;
	}

	bool Obstacle::is_intersecting_ellipsoid(const Ellipsoid&) const
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

	double Obstacle::get_repulsion_factor(const Agent& agent, double factor) const
	{
		return factor / this->compute_distance_to_point(agent.get_position(), agent.get_safety_margin());
	}

	Eigen::MatrixXd Obstacle::sample_from_parameterization(unsigned int, bool) const
	{
		std::cerr << "Fonction sample_from_parameterization of abstract class obstacle used" << std::endl;
		return Eigen::Vector3d::Zero();
	}

	bool Obstacle::point_is_inside(const Eigen::Vector3d&) const
	{
		std::cerr << "Fonction point_is_inside of abstract class obstacle used" << std::endl;
		return false;
	}

	Eigen::Vector3d Obstacle::compute_repulsion_vector(const Agent&) const
	{
		std::cerr << "Fonction compute_repulsion_vector of abstract class obstacle used" << std::endl;
		return Eigen::Vector3d::Zero();
	}

	Eigen::Vector3d Obstacle::generate_repulsion(const Agent& agent, double repulsion_threshold) const
	{
		return (this->compute_distance_to_agent(agent) < repulsion_threshold) ? compute_repulsion_vector(agent) : Eigen::Vector3d::Zero();
	}
}

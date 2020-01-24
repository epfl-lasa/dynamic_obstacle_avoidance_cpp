#include "DynamicObstacleAvoidance/Environment.hpp"

namespace DynamicObstacleAvoidance
{
	Environment::Environment()
	{}

	void Environment::add_obstacle(const std::shared_ptr<Obstacle>& obstacle)
	{
		this->insert({obstacle->get_name(), obstacle});
	}

	const std::deque<std::shared_ptr<Obstacle> > Environment::get_obstacle_list() const
	{
		std::deque<std::shared_ptr<Obstacle> > list;
		transform(this->begin(), this->end(), back_inserter(list), [](const Environment::value_type& val){return val.second;} );
		return list;
	}
}
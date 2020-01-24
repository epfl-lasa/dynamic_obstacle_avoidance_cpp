#include "DynamicObstacleAvoidance/Environment.hpp"

namespace DynamicObstacleAvoidance
{
	Environment::Environment(bool aggregated):
	aggregated(aggregated)
	{}

	void Environment::add_obstacle(const std::shared_ptr<Obstacle>& obstacle)
	{
		this->insert({obstacle->get_name(), obstacle});
	}

	const std::deque<std::shared_ptr<Obstacle> > Environment::get_obstacle_list() const
	{
		std::deque<std::shared_ptr<Obstacle> > list;
		transform(this->begin(), this->end(), back_inserter(list), [](const Environment::value_type& val){return val.second;} );
		if(this->aggregated)
		{
			list = Aggregation::aggregate_obstacles(list);
		}
		return list;
	}
}
#include "DynamicObstacleAvoidance/Environment.hpp"

namespace DynamicObstacleAvoidance
{
	Environment::Environment(bool aggregated):
	aggregated(aggregated)
	{}

	void Environment::add_obstacle(const std::shared_ptr<Obstacle>& obstacle)
	{
		this->insert({obstacle->get_name(), obstacle});
		this->update();
	}

	void Environment::update()
	{
		std::deque<std::shared_ptr<Obstacle> > list;
		transform(this->begin(), this->end(), back_inserter(list), [](const Environment::value_type& val){return val.second;} );
		this->obstacle_list = this->aggregated ? Aggregation::aggregate_obstacles(list) : list;
	}

	std::shared_ptr<Obstacle>& Environment::operator[] (const std::string& k)
	{
		auto it = this->find(k);
		if(it == this->end())
		{
			throw Exceptions::ObstacleNotInEnvironmentException(k);
		}
		return it->second;
	}
}
#include "DynamicObstacleAvoidance/Environment.hpp"
#include "DynamicObstacleAvoidance/Exceptions/ObstacleNotInEnvironmentException.hpp"
#include "DynamicObstacleAvoidance/Exceptions/ObstacleAlreadyExistsException.hpp"

namespace DynamicObstacleAvoidance
{
	Environment::Environment(bool aggregated):
	aggregated(aggregated)
	{}

	void Environment::add_obstacle(const std::shared_ptr<Obstacle>& obstacle)
	{
		auto it = this->obstacle_map.find(obstacle->get_name());
		if(it != this->obstacle_map.end())
		{
			throw Exceptions::ObstacleAlreadyExistsException(obstacle->get_name());
		}
		this->obstacle_map.insert({obstacle->get_name(), obstacle});
	}

	void Environment::remove_obstacle(const std::string& obstacle_name)
	{
		// simple case not an aggregation
		/*if(!this->is_aggregated() or (obstacle_name.find("aggregate") == std::string::npos))
		{*/
		auto it = this->obstacle_map.find(obstacle_name);
		if(it == this->obstacle_map.end())
		{
			throw Exceptions::ObstacleNotInEnvironmentException(obstacle_name);
		}
		this->obstacle_map.erase(it);
		//}
		// more complex, looks for all the primitives composing the aggregate
		/*else
		{
			// loop through the whole list of obstacles
			for( auto it = this->obstacle_map.begin(); it != this->obstacle_map.end(); ) 
			{
				// if the name of the obstacle is in the name of the aggregate remove it
				if (obstacle_name.find("_" + it->second->get_name() + "_")) it = this->obstacle_map.erase(it);
      			else ++it;
			}
		}*/
	}

	void Environment::update()
	{
		std::deque<std::shared_ptr<Obstacle> > list;
		transform(this->obstacle_map.begin(), this->obstacle_map.end(), back_inserter(list), [](const std::map<std::string, std::shared_ptr<Obstacle> >::value_type& val){return val.second;} );
		this->obstacle_list = this->aggregated ? Aggregation::aggregate_obstacles(list) : list;
	}

	std::shared_ptr<Obstacle>& Environment::operator[] (const std::string& k)
	{
		auto it = this->obstacle_map.find(k);
		if(it == this->obstacle_map.end())
		{
			// only check for aggregates
			if(this->is_aggregated())
			{
				for(auto& o:this->obstacle_list)
				{
					if(o->get_name() == k) return o;
				}
			}
			throw Exceptions::ObstacleNotInEnvironmentException(k);
		}
		return it->second;
	}


}
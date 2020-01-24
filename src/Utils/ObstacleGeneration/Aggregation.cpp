#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"

namespace DynamicObstacleAvoidance
{
	std::deque<std::shared_ptr<Obstacle> > Aggregation::aggregate_obstacles(const std::deque<std::shared_ptr<Obstacle> >& obstacles)
	{
		std::deque<std::shared_ptr<Obstacle> > aggregated_obstacles;
		std::vector<int> aggregate_indexes(obstacles.size());
		for(unsigned int i=0; i < obstacles.size(); ++i) aggregate_indexes[i] = -1;

		// first create all aggregations
		for(unsigned int i=0; i < obstacles.size(); ++i)
		{
			for(unsigned int j=i+1; j < obstacles.size(); ++j)
			{
				if(obstacles[i]->is_intersecting(*obstacles[j]))
				{	
					if(aggregate_indexes[i] != -1 && aggregate_indexes[j] == -1)
					{
						static_cast<Aggregate*>(aggregated_obstacles[aggregate_indexes[i]].get())->add_primitive(obstacles[j], false);
						aggregate_indexes[j] = aggregate_indexes[i];
					}
					else if(aggregate_indexes[j] != -1 && aggregate_indexes[i] == -1)
					{
						static_cast<Aggregate*>(aggregated_obstacles[aggregate_indexes[j]].get())->add_primitive(obstacles[i], false);
						aggregate_indexes[i] = aggregate_indexes[j];
					}
					else
					{
						if(aggregate_indexes[i] == -1 && aggregate_indexes[j] == -1)
						{
							auto ptrAggregate = std::make_shared<Aggregate>();
							ptrAggregate->add_primitive(obstacles[i]);
							ptrAggregate->add_primitive(obstacles[j]);
							int index = aggregated_obstacles.size();
							aggregate_indexes[i] = index;
							aggregate_indexes[j] = index;
							aggregated_obstacles.push_back(ptrAggregate);
						}
						else if(aggregate_indexes[i] != aggregate_indexes[j])
						{
							int old_index;
							int new_index;
							if(aggregate_indexes[i] < aggregate_indexes[j])
							{
								old_index = aggregate_indexes[j];
								new_index = aggregate_indexes[i];
							}
							else
							{
								old_index = aggregate_indexes[i];
								new_index = aggregate_indexes[j];
							}

							for(unsigned int k=0; k < obstacles.size(); ++k)
							{
								if(aggregate_indexes[k] == old_index)
								{
									static_cast<Aggregate*>(aggregated_obstacles[new_index].get())->add_primitive(obstacles[k], false);
									aggregate_indexes[k] = new_index;
								}
								else if(aggregate_indexes[k] > old_index)
								{
									aggregate_indexes[k] -= 1;
								}
							}
							aggregated_obstacles.erase(aggregated_obstacles.begin() + old_index);
						}
					}
				}
			}
		}

		// update all star hulls
		for(auto &o: aggregated_obstacles) static_cast<Aggregate&>(*o).update_hull();

		// finally add all remaining obstacles
		for(unsigned int i=0; i < aggregate_indexes.size(); ++i)
		{
			if(aggregate_indexes[i] == -1) aggregated_obstacles.push_back(obstacles[i]);
		}
		return aggregated_obstacles;
	}
}

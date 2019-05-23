#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"

std::deque<std::unique_ptr<Obstacle> > Aggregation::aggregate_obstacles(const std::deque<std::unique_ptr<Obstacle> >& obstacles)
{
	std::deque<std::unique_ptr<Obstacle> > aggregated_obstacles;
	Eigen::ArrayXi aggregate_indexes(obstacles.size());
	aggregate_indexes -= 1;

	// first create all aggregations
	for(unsigned int i=0; i<obstacles.size(); ++i)
	{
		for(unsigned int j=i+1; j<obstacles.size(); ++j)
		{
			if(obstacles[i]->is_intersecting(*obstacles[j]))
			{	
				if(aggregate_indexes(i) != -1)
				{
					dynamic_cast<Aggregate*>(aggregated_obstacles[aggregate_indexes(i)].get())->add_primitive(obstacles[j]);
					aggregate_indexes(j) = aggregate_indexes(i);
				}
				else if(aggregate_indexes(j) != -1)
				{
					dynamic_cast<Aggregate*>(aggregated_obstacles[aggregate_indexes(j)].get())->add_primitive(obstacles[i]);
					aggregate_indexes(i) = aggregate_indexes(j);
				}
				else
				{
					auto ptrAggregate = std::make_unique<Aggregate>();
					ptrAggregate->add_primitive(obstacles[i]);
					ptrAggregate->add_primitive(obstacles[j]);
					int index = aggregated_obstacles.size();
					aggregate_indexes(i) = index;
					aggregate_indexes(j) = index;
					aggregated_obstacles.push_back(std::move(ptrAggregate));
				}
			}
		}
	}

	// finally add all remaining obstacles
	for(unsigned int i=0; i<aggregate_indexes.size(); ++i)
	{
		if(aggregate_indexes(i) == -1) aggregated_obstacles.push_back(obstacles[i]->clone());
	}
	return aggregated_obstacles;
}

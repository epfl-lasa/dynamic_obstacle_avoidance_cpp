#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"

std::deque<std::unique_ptr<Obstacle> > Aggregation::aggregate_obstacles(const std::deque<std::unique_ptr<Obstacle> >& obstacles)
{
	std::deque<std::unique_ptr<Obstacle> > aggregated_obstacles;
	std::vector<int> aggregate_indexes(obstacles.size());
	for(unsigned int i=0; i<obstacles.size(); ++i) aggregate_indexes[i] = -1;

	// first create all aggregations
	for(unsigned int i=0; i<obstacles.size(); ++i)
	{
		for(unsigned int j=i+1; j<obstacles.size(); ++j)
		{
			if(obstacles[i]->is_intersecting(*obstacles[j]))
			{	
				if(aggregate_indexes[i] != -1 && aggregate_indexes[j] == -1)
				{
					static_cast<Aggregate*>(aggregated_obstacles[aggregate_indexes[i]].get())->add_primitive(obstacles[j]);
					aggregate_indexes[j] = aggregate_indexes[i];
				}
				else if(aggregate_indexes[j] != -1 && aggregate_indexes[i] == -1)
				{
					static_cast<Aggregate*>(aggregated_obstacles[aggregate_indexes[j]].get())->add_primitive(obstacles[i]);
					aggregate_indexes[i] = aggregate_indexes[j];
				}
				else
				{
					if(aggregate_indexes[i] == -1 && aggregate_indexes[j] == -1)
					{
						auto ptrAggregate = std::make_unique<Aggregate>();
						ptrAggregate->add_primitive(obstacles[i]);
						ptrAggregate->add_primitive(obstacles[j]);
						int index = aggregated_obstacles.size();
						aggregate_indexes[i] = index;
						aggregate_indexes[j] = index;
						aggregated_obstacles.push_back(std::move(ptrAggregate));
					}
					else if(aggregate_indexes[i] != aggregate_indexes[j])
					{
						std::cerr << i << ", " << j << std::endl;
						std::cerr << aggregate_indexes[i] << ", " << aggregate_indexes[j] << std::endl;
						// merge two aggregate scenario
						std::cerr << "Merging two aggregate not implemented yet" << std::endl;
					}
				}
			}
		}
	}

	// finally add all remaining obstacles
	for(unsigned int i=0; i<aggregate_indexes.size(); ++i)
	{
		if(aggregate_indexes[i] == -1) aggregated_obstacles.push_back(obstacles[i]->clone());
	}
	return aggregated_obstacles;
}

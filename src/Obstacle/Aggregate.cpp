#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"

Aggregate::Aggregate(std::deque<std::unique_ptr<Obstacle> >& primitives)
{
	// set the center as the baricenter and orientation to identity
	Eigen::Vector3d position;
	for(auto& o:primitives)
	{
		position += o->get_position();
		this->primitives.push_back(std::move(o));
	}
	position /= this->primitives.size();
	this->set_position(position);
	this->set_reference_position(position);
}

Eigen::Vector3d Aggregate::compute_normal_to_agent(const Agent& agent) const
{
	double min_dist = std::numeric_limits<double>::max();
	int index;
	int i = 0;
	for(auto& o:primitives)
	{
		double tmp_dist = o->compute_distance_to_agent(agent);
		if(tmp_dist < min_dist)
		{
			min_dist = tmp_dist;
			index = i;
		}
		++i;
	}
	return this->primitives[index]->compute_normal_to_agent(agent);
}

double Aggregate::compute_distance_to_agent(const Agent& agent) const
{
	double min_dist = std::numeric_limits<double>::max();
	for(auto& o:this->primitives)
	{
		double tmp_dist = o->compute_distance_to_agent(agent);
		min_dist = (tmp_dist < min_dist) ? tmp_dist : min_dist;
	}
	return min_dist;
}

void Aggregate::draw() const
{
	for(auto& o:this->primitives)
	{
		o->draw();
	}
	plt::plot({this->get_reference_position()(0)}, {this->get_reference_position()(1)}, "bx");
}
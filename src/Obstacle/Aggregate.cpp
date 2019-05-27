#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"

Aggregate::Aggregate()
{
	this->set_type("Aggregate");
}

Aggregate::Aggregate(const std::deque<std::unique_ptr<Obstacle> >& primitives)
{
	this->set_type("Aggregate");
	// set the center as the baricenter and orientation to identity
	Eigen::Vector3d position;
	for(auto& o:primitives)
	{
		position += o->get_position();
		this->primitives.push_back(o->clone());
	}
	position /= this->primitives.size();
	this->set_position(position);
	this->set_reference_position(position);
	for(auto& o:this->primitives)
	{
		o->set_reference_position(position);
	}
}

void Aggregate::update_positions()
{
	Eigen::Vector3d position;
	for(auto& o:this->primitives)
	{
		position += o->get_position();
	}
	position /= this->primitives.size();
	this->set_position(position);
	this->set_reference_position(position);
	for(auto& o:this->primitives)
	{
		o->set_reference_position(position);
	}
}

void Aggregate::add_primitive(const std::unique_ptr<Obstacle>& primitive)
{
	this->primitives.push_back(primitive->clone());
	this->update_positions();
}

void Aggregate::add_primitive(const Obstacle& primitive)
{
	this->primitives.push_back(primitive.clone());
	this->update_positions();
}

const Obstacle& Aggregate::get_active_obstacle(const Agent& agent) const
{
	double min_dist = std::numeric_limits<double>::max();
	int index;
	int i = 0;
	for(auto& o:this->primitives)
	{
		double tmp_dist = o->compute_distance_to_agent(agent);
		if(tmp_dist < min_dist)
		{
			min_dist = tmp_dist;
			index = i;
		}
		++i;
	}
	return *this->primitives[index];
}

Eigen::Vector3d Aggregate::compute_normal_to_agent(const Agent& agent) const
{
	double min_dist = std::numeric_limits<double>::max();
	int index;
	int i = 0;
	for(auto& o:this->primitives)
	{
		double tmp_dist = o->compute_distance_to_agent(agent);
		if(tmp_dist < min_dist)
		{
			min_dist = tmp_dist;
			index = i;
		}
		++i;
	}
	// put the normal in the aggregate frame
	Eigen::Vector3d normal = this->primitives[index]->compute_normal_to_agent(agent);
	//normal = this->get_pose().inverse() * this->primitives[index]->get_pose() * normal; 
	return normal;
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

Aggregate* Aggregate::implicit_clone() const
{}
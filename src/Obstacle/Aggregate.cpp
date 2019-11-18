#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"

namespace DynamicObstacleAvoidance
{
	Aggregate::Aggregate():
	inside_hull(true), outside_hull(false)
	{
		this->set_type("Aggregate");
	}

	Aggregate::Aggregate(const std::deque<std::unique_ptr<Obstacle> >& primitives):
	inside_hull(true), outside_hull(false)
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
		this->update_hull();
	}

	void Aggregate::update_positions()
	{
		Eigen::Vector3d position = Eigen::Vector3d::Zero();
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

	void Aggregate::update_hull()
	{
		this->outside_hull.compute_from_primitives(this->primitives, this->get_reference_position());
	}

	void Aggregate::add_primitive(const std::unique_ptr<Obstacle>& primitive, bool update_hull)
	{
		this->primitives.push_back(primitive->clone());
		this->update_positions();
		if(update_hull) this->update_hull();
	}

	void Aggregate::add_primitive(const Obstacle& primitive, bool update_hull)
	{
		this->primitives.push_back(primitive.clone());
		this->update_positions();
		if(update_hull) this->update_hull();
	}

	Eigen::Vector3d Aggregate::compute_normal_to_agent(const Agent& agent) const
	{
		return this->outside_hull.compute_normal_to_agent(agent);
	}

	double Aggregate::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{
		return this->outside_hull.compute_distance_to_point(point, safety_margin);
	}

	void Aggregate::draw(const std::string& color) const
	{
		for(auto& o:this->primitives)
		{
			o->draw(color);
		}
		plt::plot({this->get_reference_position()(0)}, {this->get_reference_position()(1)}, "bx");
		this->outside_hull.draw();
	}

	Aggregate* Aggregate::implicit_clone() const
	{}	
}
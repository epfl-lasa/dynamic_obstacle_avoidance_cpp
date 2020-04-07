#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"

namespace DynamicObstacleAvoidance
{
	Aggregate::Aggregate():
	Obstacle("aggregate"), inside_hull(true), outside_hull(false)
	{
		this->set_type("Aggregate");
	}

	Aggregate::Aggregate(const std::deque<std::shared_ptr<Obstacle> >& primitives):
	Obstacle("aggregate"), inside_hull(true), outside_hull(false)
	{
		std::string name = "aggregate";
		this->set_type("Aggregate");
		// set the center as the baricenter and orientation to identity
		Eigen::Vector3d position;
		for(auto& o:primitives)
		{
			position += o->get_position();
			this->primitives.push_back(o);
			name += ("_" + o->get_name());
		}
		this->set_name(name);
		position /= this->primitives.size();
		this->set_position(position);
		this->set_reference_position(position);
		this->update_hull();
	}

	void Aggregate::update_center_position()
	{
		Eigen::Vector3d position = Eigen::Vector3d::Zero();
		for(auto& o:this->primitives)
		{
			position += o->get_position();
		}
		position /= this->primitives.size();
		this->set_position(position);
		this->set_reference_position(position);
	}

	void Aggregate::update_hull()
	{
		this->inside_hull.compute_from_primitives(this->primitives, this->get_reference_position());
		this->outside_hull.compute_from_primitives(this->primitives, this->get_reference_position());
	}

	void Aggregate::add_primitive(const std::shared_ptr<Obstacle>& primitive, bool update_hull)
	{
		this->set_name(this->get_name() + "_" + primitive->get_name());
		this->primitives.push_back(primitive);
		this->update_center_position();
		if(update_hull) this->update_hull();
	}

	Eigen::Vector3d Aggregate::compute_normal_to_agent(const Agent& agent) const
	{
		if(this->point_is_inside(agent.get_position()))
		{
			return this->inside_hull.compute_normal_to_agent(agent);
		}
		else
		{
			return this->outside_hull.compute_normal_to_agent(agent);
		}
	}

	double Aggregate::compute_distance_to_point(const Eigen::Vector3d& point, const Eigen::Array3d& safety_margin) const
	{
		if(this->point_is_inside(point))
		{
			return this->inside_hull.compute_distance_to_point(point, safety_margin);
		}
		else
		{
			return this->outside_hull.compute_distance_to_point(point, safety_margin);
		}
	}

	void Aggregate::draw(const std::string& color, const std::string& axis) const
	{
		for(auto& o:this->primitives)
		{
			o->draw(color, axis);
		}
		plt::plot({this->get_reference_position()(0)}, {this->get_reference_position()(1)}, "bx");
		this->outside_hull.draw("k", axis);
		this->inside_hull.draw("k", axis);
	}

	bool Aggregate::point_is_inside(const Eigen::Vector3d& point) const
	{
		bool inside = this->outside_hull.point_is_inside(point);
		if(!inside) for(auto& o:this->primitives) inside |= o->point_is_inside(point);
		return inside;
	}

	Eigen::Vector3d Aggregate::compute_repulsion_vector(const Agent& agent) const
	{
		Eigen::Vector3d repulsion = this->get_repulsion_factor(agent, 20) * (agent.get_position() - this->get_reference_position());
		return this->point_is_inside(agent.get_position()) ? -repulsion : repulsion;
	}
}
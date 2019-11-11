#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"

namespace DynamicObstacleAvoidance
{
	Aggregate::Aggregate()
	{
		this->set_type("Aggregate");
		this->nb_samples=10;
		this->intersec_factor=10;
		this->inside_factor=1;
		this->distance_factor=0;
		this->area_factor=1;
	}

	Aggregate::Aggregate(const std::deque<std::unique_ptr<Obstacle> >& primitives)
	{
		this->nb_samples=10;
		this->intersec_factor=1;
		this->inside_factor=1;
		this->distance_factor=1;
		this->area_factor=1;

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
		this->hull.compute_from_primitives(this->primitives, this->get_reference_position());
	}

	void Aggregate::add_primitive(const std::unique_ptr<Obstacle>& primitive)
	{
		this->primitives.push_back(primitive->clone());
		this->update_positions();
		this->update_hull();
	}

	void Aggregate::add_primitive(const Obstacle& primitive)
	{
		this->primitives.push_back(primitive.clone());
		this->update_positions();
		this->update_hull();
	}

	/*const Obstacle& Aggregate::get_active_obstacle(const Agent& agent) const
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
		return *(this->primitives[index]);
	}*/

	Eigen::Vector3d Aggregate::compute_normal_to_agent(const Agent& agent) const
	{
		return this->hull.compute_normal_to_agent(agent);
	}

	double Aggregate::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{
		return this->hull.compute_distance_to_point(point, safety_margin);
	}

	void Aggregate::draw(const std::string& color) const
	{
		for(auto& o:this->primitives)
		{
			o->draw(color);
		}
		plt::plot({this->get_reference_position()(0)}, {this->get_reference_position()(1)}, "bx");
		this->hull.draw();
	}

	Aggregate* Aggregate::implicit_clone() const
	{}	
}
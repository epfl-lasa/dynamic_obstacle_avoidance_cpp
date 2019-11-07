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
		return *(this->primitives[index]);
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

	void Aggregate::draw(const std::string& color) const
	{
		for(auto& o:this->primitives)
		{
			o->draw(color);
		}
		plt::plot({this->get_reference_position()(0)}, {this->get_reference_position()(1)}, "bx");
	}

	Aggregate* Aggregate::implicit_clone() const
	{}

	double Aggregate::cost_function(const dlib::matrix<double,0,1>& x)
	{
		Eigen::Vector3d pos1(x(0), x(1), 0.);
		Eigen::Quaterniond rot1(Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()));
		Ellipsoid e1(State(pos1, rot1));
		e1.set_axis_lengths(Eigen::Array3d(x(3), x(4), 0));

		Eigen::Vector3d pos2(x(5), x(6), 0.);
		Eigen::Quaterniond rot2(Eigen::AngleAxisd(x(7), Eigen::Vector3d::UnitZ()));
		Ellipsoid e2(State(pos2, rot2));
		e2.set_axis_lengths(Eigen::Array3d(x(8), x(9), 0));

		double cost = this->cost_star_shape_hull(e1, e2);

		std::cout << cost << std::endl;
		return cost;
	}

	double Aggregate::cost_star_shape_hull(const Ellipsoid& e1, const Ellipsoid& e2)
	{
		double c0 = e1.is_intersecting(e2) ? 0 : 1;
		double c1 = 0;
		double c2 = 0;
		for(auto &obs_it :this->get_primitives())
		{
			Eigen::MatrixXd samples = obs_it->sample_from_parameterization(nb_samples, true);
			for(unsigned int i = 0; i < this->nb_samples; ++i)
			{
				//c1 += (e1.is_inside(samples.col(i)) || e2.is_inside(samples.col(i))) ? 0 : 1;
				double d_e1 = e1.compute_distance_to_point(samples.col(i));
				double d_e2 = e2.compute_distance_to_point(samples.col(i));
				c1 += (d_e1 - 1) * (d_e1 - 1) +  (d_e2 - 1) * (d_e2 - 1);
			}
			c2 += (e1.get_position() - obs_it->get_position()).norm();
			c2 += (e2.get_position() - obs_it->get_position()).norm();
		}
		double c3 = e1.area() + e2.area();
		return this->intersec_factor * c0 + this->inside_factor * c1 + this->distance_factor * c2 + this->area_factor * c3;
	}

	std::pair<Ellipsoid, Ellipsoid> Aggregate::compute_star_shape_hull()
	{
		Eigen::Vector3d pos = this->get_position();
		dlib::matrix<double,0,1> starting_point = {MathTools::rand_float(-10,10), MathTools::rand_float(-10,10), MathTools::rand_float(-M_PI,M_PI), 5., 5.,
			                                       MathTools::rand_float(-10,10), MathTools::rand_float(-10,10), MathTools::rand_float(-M_PI,M_PI), 5., 5.};
		dlib::matrix<double,0,1> lower_bounds = {-10., -10., -M_PI, 0.1, 0.1, -10., -10., -M_PI, 0.1, 0.1};
		dlib::matrix<double,0,1> upper_bounds = {10., 10., M_PI, 5., 5., 10., 10., M_PI, 5., 5.};

	/*	Eigen::Vector3d pos = this->get_position();
		dlib::matrix<double,0,1> starting_point = {0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1};*/


		dlib::find_min_box_constrained(dlib::lbfgs_search_strategy(10),
	                                   dlib::objective_delta_stop_strategy(1e-7),
	                                   std::bind(&Aggregate::cost_function, this, _1),
	                                   dlib::derivative(std::bind(&Aggregate::cost_function, this, _1)),
	                                   starting_point, lower_bounds, upper_bounds);


		Eigen::Vector3d pos1(starting_point(0), starting_point(1), 0.);
		Eigen::Quaterniond rot1(Eigen::AngleAxisd(starting_point(2), Eigen::Vector3d::UnitZ()));
		Ellipsoid e1(State(pos1, rot1));
		e1.set_axis_lengths(Eigen::Array3d(starting_point(3), starting_point(4), 0));

		Eigen::Vector3d pos2(starting_point(5), starting_point(6), 0.);
		Eigen::Quaterniond rot2(Eigen::AngleAxisd(starting_point(7), Eigen::Vector3d::UnitZ()));
		Ellipsoid e2(State(pos2, rot2));
		e2.set_axis_lengths(Eigen::Array3d(starting_point(8), starting_point(9), 0));

		return std::make_pair(e1, e2);
	}
}
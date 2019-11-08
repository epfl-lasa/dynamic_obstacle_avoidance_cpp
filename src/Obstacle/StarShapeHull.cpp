#include "DynamicObstacleAvoidance/Obstacle/StarShapeHull.hpp"

namespace DynamicObstacleAvoidance
{
	StarShapeHull::StarShapeHull(unsigned int resolution, const std::string& name):
	Obstacle(name), resolution(resolution)
	{
		this->set_type("StarShapeHull");
		this->set_resolution(resolution);
	}

	StarShapeHull::StarShapeHull(const std::deque<std::unique_ptr<Obstacle> >& primitives, unsigned int resolution, const std::string& name):
	Obstacle(name)
	{
		this->set_type("StarShapeHull");
		this->set_resolution(resolution);
		this->compute_from_primitives(primitives);
	}

	Eigen::Vector3d StarShapeHull::compute_baricenter(const std::deque<std::unique_ptr<Obstacle> >& primitives)
    {
        Eigen::Vector3d result = Eigen::Vector3d::Zero();
        for(auto& o:primitives)
        {
            result += o->get_position();
        }
        result /= primitives.size();
        return result;
    }

    void StarShapeHull::compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, double threshold, double min_radius, unsigned int window_size)
    {
    	// first set the reference point
		Eigen::Vector3d reference_point = this->compute_baricenter(primitives);
		// then compute the hull
		this->compute_from_primitives(primitives, reference_point, threshold, min_radius, window_size);
	}

	void StarShapeHull::compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, Eigen::Vector3d reference_point, double threshold, double min_radius, unsigned int window_size)
	{
		this->set_reference_position(reference_point);
		this->set_position(reference_point);

		std::vector<double> theta = MathTools::linspace(-M_PI, M_PI, this->resolution);
		Eigen::MatrixXd cluster_points(3, primitives.size() * this->resolution);
		unsigned int k = 0;
		for(auto& o:primitives)
		{
			Eigen::MatrixXd surface_points = o->sample_from_parameterization(this->resolution, true);
			for(unsigned int i = 0; i < surface_points.cols(); i++)
			{
			 	Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * surface_points.col(i));
			 	cluster_points.col(k * this->resolution + i) = polar_point;
			}
			++k;
		}
		// sort by ascending order of theta
		Eigen::MatrixXd sorted_cluster_points = MathTools::sorted_cols_by_theta(cluster_points);
		// recreate a star shape hull using polar coordinates
		k = 0;
		for(unsigned int i = 0; i < this->resolution; ++i)
		{
			std::vector<double> radiuses;
			while(k < sorted_cluster_points.cols() and sorted_cluster_points.col(k)(1) < theta[i])
			{
				radiuses.push_back(sorted_cluster_points.col(k)(0));
				++k;
			}
			for(unsigned int j = 0; j < window_size; ++j)
			{
				unsigned int idx = (k + j - window_size /2) % sorted_cluster_points.cols();
				if(abs(sorted_cluster_points.col(idx)(1) - theta[i]) < threshold) radiuses.push_back(sorted_cluster_points.col(idx)(0));
			}
			double radius = (!radiuses.empty()) ? *std::max_element(std::begin(radiuses), std::end(radiuses)) : min_radius;
			radius = (i > 0) ? 0.8 * radius + 0.2 * polar_surface_points.col(i-1)(0) : radius;
			Eigen::Vector3d polar_point(radius, theta[i], 0);
			this->polar_surface_points.col(i) = polar_point;
			this->cartesian_surface_points.col(i) = this->get_pose() * MathTools::polar_to_cartesian(polar_point);
		}
	}

	Eigen::Vector3d StarShapeHull::compute_normal_to_agent(const Agent& agent) const
	{
		Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * agent.get_position());
		auto surface_points = MathTools::find_closest_points(this->polar_surface_points, polar_point, 1);
		Eigen::Vector3d p1 = MathTools::polar_to_cartesian(surface_points.first);
		Eigen::Vector3d p2 = MathTools::polar_to_cartesian(surface_points.second); 
	}

	double StarShapeHull::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{
		Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * point);
		Eigen::Vector3d surface_point = MathTools::find_closest_points(this->polar_surface_points, polar_point, 1).first;
		return polar_point(0) - (surface_point(0) + safety_margin);
	}

	void StarShapeHull::draw(const std::string& color) const
	{
		std::vector<double> x;
		std::vector<double> y;

		for (unsigned int i=0; i<this->resolution; ++i)
		{
			x.push_back(this->cartesian_surface_points.col(i)(0));
			y.push_back(this->cartesian_surface_points.col(i)(1));
		}
		plt::plot(x, y, color + "-");

		if(this->get_name() == "")
		{
			plt::plot({this->get_position()(0)}, {this->get_position()(1)}, color + "o");
		}
		else
		{
			plt::text(this->get_position()(0), this->get_position()(1), this->get_name());
		}
		plt::plot({this->get_reference_position()(0)}, {this->get_reference_position()(1)}, color + "x");
	}
}

#include "DynamicObstacleAvoidance/Obstacle/StarShapeHull.hpp"

namespace DynamicObstacleAvoidance
{
	StarShapeHull::StarShapeHull(const std::string& name):
	Obstacle(name)
	{
		this->set_type("StarShapeHull");
	}

	StarShapeHull::StarShapeHull(const std::deque<std::unique_ptr<Obstacle> >& primitives, unsigned int resolution, const std::string& name):
	Obstacle(name), resolution(resolution), surface_points(Eigen::MatrixXd::Zero(3, resolution))
	{
		this->set_type("StarShapeHull");
		this->set_position(this->compute_baricenter(primitives));
		this->set_reference_position(this->get_position());
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

	void StarShapeHull::compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, double threshold, double min_radius)
	{
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
			unsigned int w = 100;
			while(k < sorted_cluster_points.cols() and sorted_cluster_points.col(k)(1) < theta[i])
			{
				radiuses.push_back(sorted_cluster_points.col(k)(0));
				++k;
			}

			for(unsigned int j = 0; j < w; ++j)
			{
				int idx = (k + j - w /2) % sorted_cluster_points.cols();
				if(abs(sorted_cluster_points.col(idx)(1) - theta[i]) < threshold) radiuses.push_back(sorted_cluster_points.col(idx)(0));
			}
			double radius = (!radiuses.empty()) ? *std::max_element(std::begin(radiuses), std::end(radiuses)) : min_radius;
			Eigen::Vector3d polar_point(radius, theta[i], 0);
			this->surface_points.col(i) = this->get_pose() * MathTools::polar_to_cartesian(polar_point);
			++k;
		}
	}

	Eigen::Vector3d StarShapeHull::compute_normal_to_agent(const Agent& agent) const
	{}

	double StarShapeHull::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{}

	void StarShapeHull::draw(const std::string& color) const
	{
		std::vector<double> x;
		std::vector<double> y;

		for (int i=0; i<this->resolution; ++i)
		{
			x.push_back(this->surface_points.col(i)(0));
			y.push_back(this->surface_points.col(i)(1));
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

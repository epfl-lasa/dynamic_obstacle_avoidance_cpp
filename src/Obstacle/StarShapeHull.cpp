#include "DynamicObstacleAvoidance/Obstacle/StarShapeHull.hpp"

namespace DynamicObstacleAvoidance
{
	StarShapeHull::StarShapeHull(unsigned int resolution, const std::string& name):
	Obstacle(name, 0.1), resolution(resolution)
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

    void StarShapeHull::compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, double min_radius)
    {
    	// first set the reference point
		Eigen::Vector3d reference_point = this->compute_baricenter(primitives);
		// then compute the hull
		this->compute_from_primitives(primitives, reference_point, min_radius);
	}

	void StarShapeHull::compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, Eigen::Vector3d reference_point, double min_radius)
	{
		this->set_reference_position(reference_point);
		this->set_position(reference_point);

		// this -1e-4 trick is to avoid that idx 0 and end represent both the same point
		std::vector<double> phi = MathTools::linspace(-M_PI, M_PI-1e-4, this->resolution);
		
		for(unsigned int i = 0; i < this->resolution; ++i)
		{
			// create a point with polar coordinates
			Eigen::Vector3d ref_point = MathTools::polar_to_cartesian(Eigen::Vector3d(1, acos(0), phi[i]));
			std::vector<Eigen::Vector3d> intersection_points;
			for(auto& o:primitives)
			{
				// transform both reference points in the object frame
				Eigen::Vector3d x1 = o->get_pose().inverse() * this->get_reference_position();
				Eigen::Vector3d x2 = o->get_pose().inverse() * this->get_pose() * ref_point;
				// calculate the line passing by the two points
				double a = (x2(1) - x1(1)) / (x2(0) - x1(0));
				double b = x1(1) - a * x1(0);
				// set the interesection equation with the ellipsoid and solve it
				Eigen::Array3d lengths = static_cast<Ellipsoid*>(o.get())->get_axis_lengths();
				double r1squared = lengths(0) * lengths(0);
				double r2squared = lengths(1) * lengths(1);
				double A = 1 / r1squared + (a*a) / r2squared;
				double B = (2*a*b) / r2squared;
				double C = (b*b) / r2squared - 1;
				double delta = B*B - 4*A*C;
				// solutions

				if(delta >= 0)
				{
					double px1 = (-B - sqrt(delta)) / (2*A);
					Eigen::Vector3d p1(px1, a * px1 + b, 0);
					double px2 = (-B + sqrt(delta)) / (2*A);
					Eigen::Vector3d p2(px2, a * px2 + b, 0);
					// transform those back to the reference point frame and in polar coordinates
					intersection_points.push_back(MathTools::cartesian_to_polar(this->get_pose().inverse() * o->get_pose() * p1));
					intersection_points.push_back(MathTools::cartesian_to_polar(this->get_pose().inverse() * o->get_pose() * p2));
				}
			}
			Eigen::Vector3d surface_point = Eigen::Vector3d(min_radius, acos(0), phi[i]);
			if(!intersection_points.empty())
			{
				// sort the intersection points in descending radius
				std::sort(std::begin(intersection_points), std::end(intersection_points), [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs){return lhs(0) > rhs(0);});
				unsigned int k = 0;
				while(k < intersection_points.size() and ((abs(intersection_points[k](2) - phi[i]) > 1e-4) and (abs((intersection_points[k](2) - 2*M_PI) - phi[i]) > 1e-4))) ++k;
				surface_point = (k < intersection_points.size()) ? intersection_points[k] : surface_point;
			}
			surface_point += Eigen::Vector3d(this->get_safety_margin(), 0, 0);
			this->polar_surface_points.col(i) = surface_point;
			this->cartesian_surface_points.col(i) = this->get_pose() * MathTools::polar_to_cartesian(surface_point);
		}
	}

	Eigen::Vector3d StarShapeHull::compute_normal_to_agent(const Agent& agent) const
	{
		Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * agent.get_position());
		auto idx = MathTools::find_closest_points(this->polar_surface_points, polar_point, 2);
		Eigen::Vector3d p1 = this->cartesian_surface_points.col(idx.first);
		Eigen::Vector3d p2 = this->cartesian_surface_points.col(idx.second);
		return (p1-p2).cross(Eigen::Vector3d::UnitZ());
	}

	double StarShapeHull::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{
		Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * point);
		unsigned int idx = MathTools::find_closest_points(this->polar_surface_points, polar_point, 1).first;
		Eigen::Vector3d surface_point = this->polar_surface_points.col(idx);
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

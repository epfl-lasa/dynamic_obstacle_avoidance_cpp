#include "DynamicObstacleAvoidance/Obstacle/StarShapeHull.hpp"

namespace DynamicObstacleAvoidance
{
	StarShapeHull::StarShapeHull(bool is_inside, unsigned int resolution, double min_radius):
	Obstacle("", 0.2), is_inside(is_inside), min_radius(min_radius)
	{
		this->set_type("StarShapeHull");
		this->set_resolution(resolution);
		// set parameters for the regressor
		this->initialize_regressor_parameters();
	}

	StarShapeHull::StarShapeHull(const std::deque<std::unique_ptr<Obstacle> >& primitives, bool is_inside, unsigned int resolution, double min_radius):
	StarShapeHull(is_inside, resolution, min_radius)
	{
		this->compute_from_primitives(primitives);
	}

	StarShapeHull::~StarShapeHull()
	{}

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

    void StarShapeHull::compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives)
    {
    	// first set the reference point
		Eigen::Vector3d reference_point = this->compute_baricenter(primitives);
		// then compute the hull
		this->compute_from_primitives(primitives, reference_point);
	}

	void StarShapeHull::compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, Eigen::Vector3d reference_point)
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
				Eigen::Array3d lengths = static_cast<Ellipsoid*>(o.get())->get_axis_lengths() + o->get_safety_margin();
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
			Eigen::Vector3d surface_point = Eigen::Vector3d(this->min_radius, acos(0), phi[i]);
			double safety = this->is_inside ? -this->get_safety_margin() : this->get_safety_margin();
			if(!intersection_points.empty())
			{
				// sort the intersection points in descending radius
				if(this->is_inside)
				{
					// if we are computing the inside hull then we need the closest point
					std::sort(std::begin(intersection_points), std::end(intersection_points), [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs){return lhs(0) < rhs(0);});
				}
				else
				{
					// if we are computing the outside hull then we need the furthest one
					std::sort(std::begin(intersection_points), std::end(intersection_points), [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs){return lhs(0) > rhs(0);});
				}
				unsigned int k = 0;
				while(k < intersection_points.size() and ((abs(intersection_points[k](2) - phi[i]) > 1e-4) and (abs((intersection_points[k](2) - 2*M_PI) - phi[i]) > 1e-4))) ++k;
				surface_point = (k < intersection_points.size()) ? intersection_points[k] : surface_point;
			}
			surface_point(0) += safety;
			surface_point(0) = std::max(this->min_radius, surface_point(0));
			// smooth function
			if(i > 1)
			{
				this->polar_surface_points.col(i-1)(0) = 0.9 * this->polar_surface_points.col(i-1)(0) + \
				                                         0.05 * this->polar_surface_points.col(i-2)(0) + \
				                                         0.05 * this->polar_surface_points.col(i)(0);
			}
			this->polar_surface_points.col(i) = surface_point;
		}
		// smooth 0 point with the last one
		this->polar_surface_points.col(0)(0) = 0.9 * this->polar_surface_points.col(0)(0) + \
					                           0.05 * this->polar_surface_points.col(this->resolution-1)(0) + \
					                           0.05 * this->polar_surface_points.col(1)(0);
		// smooth last point
		this->polar_surface_points.col(this->resolution-1)(0) = 0.9 * this->polar_surface_points.col(this->resolution-1)(0) + \
					                           					0.05 * this->polar_surface_points.col(this->resolution-2)(0) + \
					                           					0.05 * this->polar_surface_points.col(0)(0);
		 // calculate cartesian coordinates
		for(unsigned int i = 0; i < this->resolution; ++i)
		{
			this->cartesian_surface_points.col(i) = this->get_pose() * MathTools::polar_to_cartesian(this->polar_surface_points.col(i));
		}
		// create the regressor model
		//this->train_surface_regressor();
	}

	Eigen::Vector3d StarShapeHull::compute_normal_to_agent(const Agent& agent) const
	{
		Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * agent.get_position());
		//Eigen::Vector3d surface_point1 = this->predict_cartesian_point(polar_point(2));
		//Eigen::Vector3d surface_point2 = this->predict_cartesian_point(polar_point(2)+0.01);
		
		auto idx = MathTools::find_closest_points(this->polar_surface_points, polar_point, 2);
		Eigen::Vector3d surface_point1 = this->cartesian_surface_points.col(idx.first);
		Eigen::Vector3d surface_point2 = this->cartesian_surface_points.col(idx.second);

		return (surface_point1-surface_point2).cross(Eigen::Vector3d::UnitZ());
	}

	double StarShapeHull::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{
		Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * point);
		//Eigen::Vector3d surface_point = this->predict_polar_point(polar_point(2));
		
		unsigned int idx = MathTools::find_closest_points(this->polar_surface_points, polar_point, 2).first;
		Eigen::Vector3d surface_point = this->polar_surface_points.col(idx);

		double distance;
		if(this->is_inside)
			distance = surface_point(0) / polar_point(0);
		else
			distance = polar_point(0) / surface_point(0);
		return distance;
	}

	void StarShapeHull::draw(const std::string& color) const
	{
		std::vector<double> x;
		std::vector<double> y;

		std::vector<double> phi = MathTools::linspace(-M_PI, M_PI-1e-4, this->resolution);
		for (unsigned int i=0; i<this->resolution; ++i)
		{
			Eigen::Vector3d cartesian_point = this->predict_cartesian_point(phi[i]);
			x.push_back(cartesian_point(0));
			y.push_back(cartesian_point(1));
		}

		//plt::plot(x, y, color + "--");

		x.clear();
		y.clear();
		for (unsigned int i=0; i<this->resolution; ++i)
		{		
			x.push_back(this->cartesian_surface_points.col(i)(0));
			y.push_back(this->cartesian_surface_points.col(i)(1));
		}

		plt::plot(x, y, color + "--");

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

	void StarShapeHull::initialize_regressor_parameters()
	{
		this->surface_regressor = GP_t(2, 2);
	}

	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd> > StarShapeHull::extract_regressor_data() const
	{
		std::vector<Eigen::VectorXd> samples;
		samples.resize(this->resolution);
		std::vector<Eigen::VectorXd> observations;
		observations.resize(this->resolution);

		for(unsigned int i = 0; i < this->resolution; ++i)
		{
			samples[i] = Eigen::VectorXd(2);
			samples[i] << cos(this->polar_surface_points.col(i)(2)), sin(this->polar_surface_points.col(i)(2));
			
			//samples[i] = Eigen::VectorXd(1);
			//samples[i] << surface_points.col(i)(2);

			observations[i] = Eigen::VectorXd(2);
			observations[i] << this->cartesian_surface_points.col(i)(0), this->cartesian_surface_points.col(i)(1);
		}
		return std::make_pair(samples, observations);
	}

	void StarShapeHull::train_surface_regressor()
	{
		auto data = this->extract_regressor_data();
		this->surface_regressor.compute(data.first, data.second, false);
		this->surface_regressor.optimize_hyperparams();
	}

	Eigen::Vector3d StarShapeHull::predict_cartesian_point(double angle) const
	{
		Eigen::VectorXd mu;
		double sigma;

		Eigen::VectorXd sample(2);
		sample << cos(angle), sin(angle);
		
		//Eigen::VectorXd sample(1);
		//sample << angle;

		std::tie(mu, sigma) = this->surface_regressor.query(sample);
		return Eigen::Vector3d(mu[0], mu[1], 0);
	}

	Eigen::Vector3d StarShapeHull::predict_polar_point(double angle) const
	{
		Eigen::Vector3d cartesian_point = this->predict_cartesian_point(angle);
		return MathTools::cartesian_to_polar(cartesian_point);
	}

	bool StarShapeHull::point_is_inside(const Eigen::Vector3d& point) const
	{
		Eigen::Vector3d polar_point = MathTools::cartesian_to_polar(this->get_pose().inverse() * point);
		//Eigen::Vector3d surface_point = this->predict_polar_point(polar_point(2));
		
		unsigned int idx = MathTools::find_closest_points(this->polar_surface_points, polar_point, 2).first;
		Eigen::Vector3d surface_point = this->polar_surface_points.col(idx);

		return polar_point[0] < surface_point[0];
	}

	bool StarShapeHull::is_closed() const
	{
		double min_radius_point = this->polar_surface_points.row(0).minCoeff();
		return min_radius_point > (this->min_radius + this->get_safety_margin());
	}
}

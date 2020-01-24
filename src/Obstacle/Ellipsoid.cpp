#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"

namespace DynamicObstacleAvoidance
{
	Ellipsoid::Ellipsoid(const std::string& name):
	Obstacle(name), axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
	{
		this->set_type("Ellipsoid");
	}

	Ellipsoid::Ellipsoid(const Ellipsoid& ellipsoid):
	Obstacle(ellipsoid.get_state(), ellipsoid.get_reference_position(), ellipsoid.get_safety_margin(), ellipsoid.get_name()),
	axis_lengths(ellipsoid.get_axis_lengths()), curvature_factor(ellipsoid.get_curvature_factor()), epsilon(1E-4)
	{
		this->set_type("Ellipsoid");
	}

	Ellipsoid::Ellipsoid(const State& state, double safety_margin, const std::string& name):
	Obstacle(state, safety_margin, name), axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
	{
		this->set_type("Ellipsoid");
	}

	Ellipsoid::Ellipsoid(const State& state, const Eigen::Vector3d& reference_position, double safety_margin, const std::string& name):
	Obstacle(state, reference_position, safety_margin, name), axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
	{
		this->set_type("Ellipsoid");
	}

	Ellipsoid::Ellipsoid(double cx, double cy, double cz, double safety_margin, const std::string& name):
	Obstacle(cx, cy, cz, safety_margin, name), axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
	{
		this->set_type("Ellipsoid");
	}

	Ellipsoid::~Ellipsoid()
	{}

	Eigen::Vector3d Ellipsoid::compute_normal_to_agent(const Agent& agent) const
	{
		Eigen::Array3d point_in_frame = this->get_pose().inverse() * agent.get_position();
		Eigen::Array3d lengths = this->axis_lengths + this->get_safety_margin() + agent.get_safety_margin();
		Eigen::Array3d tmp_values = (2 * this->curvature_factor * point_in_frame);
		for(int i=0; i<3; ++i)
		{
			tmp_values(i) = (lengths(i) > this->epsilon) ? tmp_values(i) / (lengths(i) * lengths(i)) : tmp_values(i);
		}
		Eigen::Vector3d normal_vector = tmp_values.pow(2 * this->curvature_factor - 1);
		normal_vector.normalize();
		return normal_vector;
	}

	double Ellipsoid::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{
		Eigen::Array3d point_in_frame = this->get_pose().inverse() * point;
		Eigen::Array3d lengths = this->axis_lengths + this->get_safety_margin() + safety_margin;
		Eigen::Array3d tmp_values;
		for(int i=0; i<3; ++i)
		{
			tmp_values(i) = (lengths(i) > this->epsilon) ? point_in_frame(i) / lengths(i) : point_in_frame(i);
		}
		tmp_values = tmp_values.pow(2 * this->curvature_factor);
		return tmp_values.sum();
	}

	void Ellipsoid::draw(const std::string& color, bool is3D) const 
	{
		unsigned int n1 = 100;
		unsigned int n2 = is3D ? n1 : 1;
		// use a linespace to have a full rotation angle between [-pi, pi]
		std::vector<double> alpha = MathTools::linspace(-M_PI/2, 3*M_PI/2, n1);

		// use the parametric equation of an ellipse to draw
		std::vector<std::vector<double> > x(n2);
		std::vector<std::vector<double> > y(n2);
		std::vector<std::vector<double> > z(n2);
		std::vector<std::vector<double> > x_safety(n2);
		std::vector<std::vector<double> > y_safety(n2);
		std::vector<std::vector<double> > z_safety(n2);
		Eigen::Array3d safety_length =  this->get_axis_lengths() + this->get_safety_margin();
		for (unsigned int i=0; i<n2; ++i)
		{
			std::vector<double> xrow(n1);
			std::vector<double> yrow(n1);
			std::vector<double> zrow(n1);
			std::vector<double> xrow_safety(n1);
			std::vector<double> yrow_safety(n1);
			std::vector<double> zrow_safety(n1);
			for (unsigned int j=0; j<n1; ++j)
			{
				double phi = alpha.at(j);
				double theta = alpha.at(i);
				Eigen::Vector3d point;
				point(0) = this->get_axis_lengths(0) * sin(theta) * cos(phi);
				point(1) = this->get_axis_lengths(1) * sin(theta) * sin(phi);
				point(2) = this->get_axis_lengths(2) * cos(theta);
				
				Eigen::Vector3d safety_point;
				safety_point(0) = safety_length(0) * sin(theta) * cos(phi);
				safety_point(1) = safety_length(1) * sin(theta) * sin(phi);
				safety_point(2) = safety_length(2) * cos(theta);

				point = this->get_pose() * point;
				safety_point = this->get_pose() * safety_point;

				xrow.at(j) = point(0);
				yrow.at(j) = point(1);
				zrow.at(j) = point(2);
				xrow_safety.at(j) = point(0);
				yrow_safety.at(j) = point(1);
				zrow_safety.at(j) = point(2);
			}
			x.at(i) = xrow;
			y.at(i) = yrow;
			z.at(i) = zrow;
			x_safety.at(i) = xrow_safety;
			y_safety.at(i) = yrow_safety;
			z_safety.at(i) = zrow_safety;
		}

		if (is3D)
		{
			plt::plot_surface(x, y, z);
			plt::plot_surface(x_safety, y_safety, z_safety);
		}
		else
		{
			plt::plot(x[0], y[0], color + "-");
			//plt::plot(x_safety[0], y_safety[0], color + "--");
		}

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

	bool Ellipsoid::point_is_inside(const Eigen::Vector3d& point) const
	{
		Eigen::Array3d lengths = this->get_axis_lengths() + this->get_safety_margin();
		Eigen::Vector3d transformed_point = this->get_pose().inverse() * point;
		double eq_value = ((transformed_point(0) * transformed_point(0)) / (lengths(0) * lengths(0))) + ((transformed_point(1) * transformed_point(1)) / (lengths(1) * lengths(1)));
		return (eq_value <= 1);
	}

	bool Ellipsoid::is_intersecting_ellipsoid(const Ellipsoid& other_obstacle) const
	{
		bool intersecting = false;
		// first check that center points satisfy equations of the other
		if(this->point_is_inside(other_obstacle.get_position()))
		{
			intersecting = true;
		}
		else if(other_obstacle.point_is_inside(this->get_position()))
		{
			intersecting = true;
		}
		else
		{
			// first sample the first ellipsoid
			unsigned int nb_samples = 100;
			Eigen::MatrixXd samples = other_obstacle.sample_from_parameterization(nb_samples, true);
			// for each points check if at least one of them is inside
			unsigned int i = 0;
			while(!intersecting && i<samples.cols())
			{
				intersecting = this->point_is_inside(samples.col(i));
				++i;
			}
		}
		return intersecting;
	}

	Eigen::MatrixXd Ellipsoid::sample_from_parameterization(unsigned int nb_samples, bool is_include_safety_margin) const
	{
		// convert quaternion to AngleAxis
		Eigen::AngleAxisd orientation(this->get_orientation());
		Eigen::Vector3d axis = orientation.axis();
		double theta = (axis(2) > 0) ? orientation.angle() :  2*M_PI - orientation.angle();
		Eigen::Array3d lengths = (is_include_safety_margin) ? this->get_axis_lengths() + this->get_safety_margin() : this->get_axis_lengths();

		// use a linespace to have a full rotation angle between [0, 2pi]
		std::vector<double> alpha = MathTools::linspace(0, 2*M_PI, nb_samples);

		Eigen::MatrixXd samples(3, nb_samples);
		for(unsigned int i=0; i<nb_samples; ++i)
		{
			double a = alpha.at(i);
			samples(0, i) = lengths(0) * cos(a) * cos(theta) - lengths(1) * sin(a) * sin(theta) + this->get_position()(0);
			samples(1, i) = lengths(0) * cos(a) * sin(theta) + lengths(1) * sin(a) * cos(theta) + this->get_position()(1);
			samples(2, i) = 0;
		}
		return samples;
	}

	double Ellipsoid::area(bool is_include_safety_margin) const
	{
		Eigen::Array3d lengths = (is_include_safety_margin) ? this->get_axis_lengths() + this->get_safety_margin() : this->get_axis_lengths();
		return 4 * M_PI * (pow((pow(lengths(0), 1.6) + pow(lengths(1), 1.6) + pow(lengths(2), 1.6))/3, 1/1.6));
	}

	Eigen::Vector3d Ellipsoid::compute_repulsion_vector(const Agent& agent) const
	{
		return this->get_repulsion_factor(agent) * (agent.get_position() - this->get_reference_position());
	}
}

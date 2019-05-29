#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"

Ellipsoid::Ellipsoid():
axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
{
	this->set_type("Ellipsoid");
}

Ellipsoid::Ellipsoid(const Ellipsoid& ellipsoid):
Obstacle(ellipsoid.get_state(), ellipsoid.get_reference_position(), ellipsoid.get_safety_margin()),
axis_lengths(ellipsoid.get_axis_lengths()), curvature_factor(ellipsoid.get_curvature_factor()), epsilon(1E-4)
{
	this->set_type("Ellipsoid");
}

Ellipsoid::Ellipsoid(const State& state, const double& safety_margin):
Obstacle(state, safety_margin), axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
{
	this->set_type("Ellipsoid");
}

Ellipsoid::Ellipsoid(const State& state, const Eigen::Vector3d& reference_position, const double& safety_margin):
Obstacle(state, reference_position, safety_margin), axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
{
	this->set_type("Ellipsoid");
}

Ellipsoid::Ellipsoid(const double& cx, const double& cy, const double& cz, const double& safety_margin):
Obstacle(cx, cy, cz, safety_margin), axis_lengths(1,1,1), curvature_factor(1,1,1), epsilon(1E-4)
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

double Ellipsoid::compute_distance_to_agent(const Agent& agent) const
{
	Eigen::Array3d point_in_frame = this->get_pose().inverse() * agent.get_position();
	Eigen::Array3d lengths = this->axis_lengths + this->get_safety_margin() + agent.get_safety_margin();
	Eigen::Array3d tmp_values;
	for(int i=0; i<3; ++i)
	{
		tmp_values(i) = (lengths(i) > this->epsilon) ? point_in_frame(i) / lengths(i) : point_in_frame(i);
	}
	tmp_values = tmp_values.pow(2 * this->curvature_factor);
	return tmp_values.sum();
}

void Ellipsoid::draw() const 
{
	int n = 100;
	// use a linespace to have a full rotation angle between [-pi, pi]
	std::vector<double> alpha = MathTools::linspace(0, 2*M_PI, n);
	// convert quaternion to AngleAxis
	Eigen::AngleAxisd orientation(this->get_orientation());
	Eigen::Vector3d axis = orientation.axis();
	double theta = (axis(2) > 0) ? orientation.angle() :  2*M_PI - orientation.angle();

	// use the parametric equation of an ellipse to draw
	std::vector<double> x(n);
	std::vector<double> y(n);
	std::vector<double> x_safety(n);
	std::vector<double> y_safety(n);
	Eigen::Array3d safety_length =  this->get_axis_lengths() + this->get_safety_margin();
	for (int i=0; i<n; ++i)
	{
		double a = alpha.at(i);
		x.at(i) = this->get_axis_lengths(0) * cos(a) * cos(theta) - this->get_axis_lengths(1) * sin(a) * sin(theta) + this->get_position()(0);
		y.at(i) = this->get_axis_lengths(0) * cos(a) * sin(theta) + this->get_axis_lengths(1) * sin(a) * cos(theta) + this->get_position()(1);

		x_safety.at(i) = safety_length(0) * cos(a) * cos(theta) - safety_length(1) * sin(a) * sin(theta) + this->get_position()(0);
		y_safety.at(i) = safety_length(0) * cos(a) * sin(theta) + safety_length(1) * sin(a) * cos(theta) + this->get_position()(1);
	}
	plt::plot(x, y);
	plt::plot(x_safety, y_safety, "k-");
	plt::plot({this->get_position()(0)}, {this->get_position()(1)}, "ko");
	plt::plot({this->get_reference_position()(0)}, {this->get_reference_position()(1)}, "kx");
}

bool Ellipsoid::is_inside(const Eigen::Vector3d& point) const
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
	if(this->is_inside(other_obstacle.get_position()))
	{
		intersecting = true;
	}
	else if(other_obstacle.is_inside(this->get_position()))
	{
		intersecting = true;
	}
	else
	{
		// first sample the first ellipsoid
		int nb_samples = 100;
		Eigen::MatrixXd samples = other_obstacle.sample_from_parameterization(nb_samples, true);
		// for each points check if at least one of them is inside
		int i = 0;
		while(!intersecting && i<samples.cols())
		{
			intersecting = this->is_inside(samples.col(i));
			++i;
		}
	}
	return intersecting;
}

Ellipsoid* Ellipsoid::implicit_clone() const
{
	return new Ellipsoid(*this);
}

Eigen::MatrixXd Ellipsoid::sample_from_parameterization(const int& nb_samples, const bool& is_include_safety_margin) const
{
	// convert quaternion to AngleAxis
	Eigen::AngleAxisd orientation(this->get_orientation());
	Eigen::Vector3d axis = orientation.axis();
	double theta = (axis(2) > 0) ? orientation.angle() :  2*M_PI - orientation.angle();
	Eigen::Array3d lengths = (is_include_safety_margin) ? this->get_axis_lengths() + this->get_safety_margin() : this->get_axis_lengths();

	// use a linespace to have a full rotation angle between [-pi, pi]
	std::vector<double> alpha = MathTools::linspace(0, 2*M_PI, nb_samples);

	Eigen::MatrixXd samples(3, nb_samples);
	for(int i=0; i<nb_samples; ++i)
	{
		double a = alpha.at(i);
		samples(0, i) = lengths(0) * cos(a) * cos(theta) - lengths(1) * sin(a) * sin(theta) + this->get_position()(0);
		samples(1, i) = lengths(0) * cos(a) * sin(theta) + lengths(1) * sin(a) * cos(theta) + this->get_position()(1);
		samples(2, i) = 0;
	}
	return samples;
}
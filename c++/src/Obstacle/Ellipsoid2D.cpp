#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid2D.hpp"

Ellipsoid2D::Ellipsoid2D(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, Eigen::Vector3f& reference_position, double safety_margin):
Obstacle(center_position, center_orientation, reference_position, safety_margin)
{
	this->direction_axis << 1, 1;
	this->lengths << 1, 1;
}

Ellipsoid2D::Ellipsoid2D(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, double safety_margin):
Ellipsoid2D(center_position, center_orientation, center_position, safety_margin)
{}

Ellipsoid2D::~Ellipsoid2D(){}

Eigen::VectorXf Ellipsoid2D::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{

}

double Ellipsoid2D::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{
	Eigen::Vector3f point_in_frame = external_point - this->get_center_position();
	Eigen::Array2f tmp_values;
	tmp_values << point_in_frame(0) / this->direction_axis(0), point_in_frame(1) / this->direction_axis(1);
	tmp_values = tmp_values.pow(2 * this->lengths);
	return tmp_values.sum();
}
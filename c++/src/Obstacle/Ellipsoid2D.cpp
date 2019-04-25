#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid2D.hpp"

Ellipsoid2D::Ellipsoid2D(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, Eigen::Vector3f& reference_position, float safety_margin):
Obstacle(center_position, center_orientation, reference_position, safety_margin)
{
	this->axis_lengths << 1, 1;
	this->curvature_factor << 1, 1;
}

Ellipsoid2D::Ellipsoid2D(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, float safety_margin):
Ellipsoid2D(center_position, center_orientation, center_position, safety_margin)
{}

Ellipsoid2D::~Ellipsoid2D(){}

Eigen::Vector3f Ellipsoid2D::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{
	Eigen::Array3f point_in_frame = external_point - this->get_center_position();
	Eigen::Vector3f normal_vector;
	Eigen::Array2f tmp_values = (2 * this->curvature_factor * point_in_frame.head(2)) / (this->axis_lengths * this->axis_lengths);
	tmp_values = tmp_values.pow(2 * this->curvature_factor - 1);
	normal_vector << tmp_values(0), tmp_values(1), 0;
	normal_vector.normalize();
	return normal_vector;
}

float Ellipsoid2D::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{
	Eigen::Array3f point_in_frame = external_point - this->get_center_position();
	Eigen::Array2f tmp_values = point_in_frame.head(2) / this->axis_lengths;
	tmp_values = tmp_values.pow(2 * this->curvature_factor);
	return tmp_values.sum();
}

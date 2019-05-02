#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid2D.hpp"

Ellipsoid2D::Ellipsoid2D(const State& state, const float& safety_margin):
Obstacle(state, safety_margin), axis_lengths(1,1), curvature_factor(1,1)
{}

Ellipsoid2D::Ellipsoid2D(const State& state, const Eigen::Vector3f& reference_position, const float& safety_margin):
Obstacle(state, reference_position, safety_margin), axis_lengths(1,1), curvature_factor(1,1)
{}

Ellipsoid2D::~Ellipsoid2D()
{}

Eigen::Vector3f Ellipsoid2D::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{
	Eigen::Array3f point_in_frame = this->get_pose().inverse() * external_point;
	Eigen::Vector3f normal_vector;
	Eigen::Array2f tmp_values = (2 * this->curvature_factor * point_in_frame.head(2)) / (this->axis_lengths * this->axis_lengths);
	tmp_values = tmp_values.pow(2 * this->curvature_factor - 1);
	normal_vector << tmp_values(0), tmp_values(1), 0;
	normal_vector.normalize();
	return normal_vector;
}

float Ellipsoid2D::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{
	Eigen::Array3f point_in_frame = this->get_pose().inverse() * external_point;
	Eigen::Array2f tmp_values = point_in_frame.head(2) / this->axis_lengths;
	tmp_values = tmp_values.pow(2 * this->curvature_factor);
	return tmp_values.sum();
}

#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"

Ellipsoid::Ellipsoid(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, Eigen::Vector3f& reference_position, double safety_margin):
Obstacle(center_position, center_orientation, reference_position, safety_margin)
{}

Ellipsoid::Ellipsoid(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, double safety_margin):
Obstacle(center_position, center_orientation, center_position, safety_margin)
{}

Ellipsoid::~Ellipsoid(){}

Eigen::VectorXf Ellipsoid::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{

}

double Ellipsoid::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{
	Eigen::Array3f tmp_values = external_point.array() / this->direction_axis.array();
	tmp_values.pow(2 * this->lengths.array());
	return tmp_values.sum();
}
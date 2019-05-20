#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"

Ellipsoid::Ellipsoid():
Obstacle(), axis_lengths(1,1,1), curvature_factor(1,1,1)
{}

Ellipsoid::Ellipsoid(State state, const double& safety_margin):
Obstacle(state, safety_margin), axis_lengths(1,1,1), curvature_factor(1,1,1)
{}

Ellipsoid::Ellipsoid(const State& state, const Eigen::Vector3d& reference_position, const double& safety_margin):
Obstacle(state, reference_position, safety_margin), axis_lengths(1,1,1), curvature_factor(1,1,1)
{}

Ellipsoid::Ellipsoid(const double& cx, const double& cy, const double& cz, const double& safety_margin):
Obstacle(cx, cy, cz, safety_margin), axis_lengths(1,1,1), curvature_factor(1,1,1)
{}

Ellipsoid::~Ellipsoid()
{}

Eigen::Vector3d Ellipsoid::compute_normal_to_agent(const Agent& agent) const
{
	Eigen::Array3d point_in_frame = this->get_pose().inverse() * agent.get_position();
	Eigen::Array3d lengths = this->axis_lengths + this->get_safety_margin() + agent.get_safety_margin();
	Eigen::Array3d tmp_values = (2 * this->curvature_factor * point_in_frame) / (lengths * lengths);
	Eigen::Vector3d normal_vector = tmp_values.pow(2 * this->curvature_factor - 1);
	normal_vector.normalize();
	return normal_vector;
}

double Ellipsoid::compute_distance_to_agent(const Agent& agent) const
{
	Eigen::Array3d point_in_frame = this->get_pose().inverse() * agent.get_position();
	Eigen::Array3d lengths = this->axis_lengths + this->get_safety_margin() + agent.get_safety_margin();
	Eigen::Array3d tmp_values = point_in_frame / lengths;
	tmp_values = tmp_values.pow(2 * this->curvature_factor);
	return tmp_values.sum();
}

std::pair<bool, Eigen::Vector3d> Ellipsoid::find_intersection_center(const Ellipsoid& other_obstacle) const
{
	std::cerr << "Fonction of ellipsoid class obstacle used" << std::endl;
}
#include "DynamicObstacleAvoidance/Agent.hpp"

Agent::Agent(const double& safety_margin):
safety_margin(safety_margin), delta_t(0.5)
{
	this->envelope = std::make_unique<Ellipsoid>(State(this->get_position()));
}

Agent::Agent(const State& state, const double& safety_margin):
state(state), safety_margin(safety_margin), delta_t(0.5)
{
	this->envelope = std::make_unique<Ellipsoid>(State(this->get_position()));
}

Agent::Agent(const Agent& agent):
state(agent.state), safety_margin(agent.safety_margin), envelope(agent.get_envelope().clone()), delta_t(agent.delta_t)
{}

void Agent::update_envelope()
{
	double margin = std::max(0.1, this->safety_margin);
	// project an ellipsoid in front of the agent based on its velocity
	Eigen::Vector3d center_position = this->get_position() + this->delta_t * this->get_linear_velocity().normalized();
	this->envelope->set_position(center_position);
	this->envelope->set_reference_position(center_position);
	static_cast<Ellipsoid*>(this->envelope.get())->set_axis_lengths(Eigen::Array3d((center_position - this->get_position()).norm()+margin, margin, margin));
	// calculate the orientation using cross product
	double cos_theta = this->get_linear_velocity().normalized().dot(Eigen::Vector3d::UnitX());
	double sin_theta = this->get_linear_velocity().normalized().dot(Eigen::Vector3d::UnitY());
	double theta = atan2(sin_theta, cos_theta);
	// for now consider 2D representation
	this->envelope->set_orientation(Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())));
}

bool Agent::in_obstacle(const Obstacle& obstacle) const
{
	double margin = std::max(0.1, this->safety_margin);
	Ellipsoid safety_envelope(State(this->get_position()));
	safety_envelope.set_axis_lengths(Eigen::Vector3d(margin, margin, margin));
	return safety_envelope.is_intersecting(obstacle);
}
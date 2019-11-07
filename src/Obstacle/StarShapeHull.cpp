#include "DynamicObstacleAvoidance/Obstacle/StarShapeHull.hpp"

namespace DynamicObstacleAvoidance
{
	StarShapeHull::StarShapeHull()
	{}

	StarShapeHull::StarShapeHull(const std::deque<std::unique_ptr<Obstacle> >& primitives)
	{}

	Eigen::Vector3d StarShapeHull::compute_normal_to_agent(const Agent& agent) const
	{}

	double StarShapeHull::compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const
	{}

	void StarShapeHull::draw(const std::string& color) const
	{}
}

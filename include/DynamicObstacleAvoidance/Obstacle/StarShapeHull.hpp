/**
 * @class StarShapeHull
 * @brief Class to define a star shape hull
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_

#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
#include <algorithm>
#include <deque>
#define _USE_MATH_DEFINES 
#include <cmath>

namespace plt = matplotlibcpp;

namespace DynamicObstacleAvoidance
{
	class StarShapeHull: public Obstacle
	{
	private:
		bool is_inside; //< true if the hull is an inside hull
		unsigned int resolution; //< the number of sample points of the hull 
		double min_radius; //< the minimum radius when there are no intersection with primitives obstacles
		Eigen::MatrixXd cartesian_surface_points; //< surface points in cartesian space
		Eigen::MatrixXd polar_surface_points; //< surface points in polar space

		/**
		 * @brief Function to compute the baricenter of the hull
		 * @param primitives the primitives on which the hull is computed
		 * @return the baricenter position
		 */
		Eigen::Vector3d compute_baricenter(const std::deque<std::shared_ptr<Obstacle> >& primitives);

	public:
		/**
		 * @brief Constructor with default parameters
		 * @param is_inside true if the hull is an inside hull
		 * @param resolution the number of sample points of the hull 
		 * @param min_radius the minimum radius when there are no intersection with primitives obstacles in percent of the maximum distance
		 */
		explicit StarShapeHull(bool is_inside=false, unsigned int resolution=500, double min_radius=0.5);

		/**
		 * @brief Constructor from the list of primitives
		 * @param primitives the list of primitives
		 * @param is_inside true if the hull is an inside hull
		 * @param resolution the number of sample points of the hull 
		 * @param min_radius the minimum radius when there are no intersection with primitives obstacles in percent of the maximum distance
		 */
		explicit StarShapeHull(const std::deque<std::shared_ptr<Obstacle> >& primitives, bool is_inside=false, unsigned int resolution=500, double min_radius=0.5);

		/**
		 * @brief Destructor
		 */
		~StarShapeHull();

		/**
		 * @brief Function to compute the normal on the surface to the agent wrt the reference point 
		 * @param agent the agent 
		 * @return the normal vector
		 */
		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		/**
		 * @brief Function to compute the distance between a point and the surface of the obstacle wrt the reference point 
		 * @param point the external point
		 * @param safety_margin the safety margin to add
		 * @return the distance value (1 if the point is on the surface)
		 */
		double compute_distance_to_point(const Eigen::Vector3d& point, const Eigen::Array3d& safety_margin) const;

		/**
		 * @brief Function to draw an agent
		 * @param color the color to apply
		 * @param axis axis to draw (default = "xy")
		 */
		void draw(const std::string& color="k", const std::string& axis="xy") const;

		/**
		 * @brief Setter of the resultion attribute
		 * @param resolution the new value
		 */
		void set_resolution(unsigned int resolution);

		/**
		 * @brief Function to compute the hull from primitives
		 * @param primitives the list of primitives obstacles
		 * @param reference_point the reference point of the hull
		 */
		void compute_from_primitives(const std::deque<std::shared_ptr<Obstacle> >& primitives, Eigen::Vector3d reference_point);

		/**
		 * @brief Function to compute the hull from primitives
		 * @param primitives the list of primitives obstacles
		 */
		void compute_from_primitives(const std::deque<std::shared_ptr<Obstacle> >& primitives);

		/**
		 * @brief Function to check if a point is inside the obstacle
		 * @param point the point
		 * @return true if the point is inside
		 */
		bool point_is_inside(const Eigen::Vector3d& point) const;

		/**
		 * @brief Function to check if the hull is closed, i.e. every surface point is farther than the min radius
		 * @return true if the hull is closed
		 */
		bool is_closed() const;
	};

	inline void StarShapeHull::set_resolution(unsigned int resolution)
	{
		this->resolution = resolution;
		this->cartesian_surface_points = Eigen::MatrixXd::Zero(3, resolution);
		this->polar_surface_points = Eigen::MatrixXd::Zero(3, resolution);
	}
}
#endif
/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_ELLIPSOID_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_ELLIPSOID_H_

#include <eigen3/Eigen/Core>
#define _USE_MATH_DEFINES 
#include <cmath>
#include <vector>
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"

namespace DynamicObstacleAvoidance
{
	class Agent;

	class Ellipsoid: public Obstacle 
	{
	private:
		Eigen::Array3d axis_lengths; //< axis lenghts in x,y,z directions
		Eigen::Array3d curvature_factor; //< curvature factor in x,y,z directions
		double epsilon; //< epsilon value for threshold functions

		/**
		 * @brief Function to check if the obstacle is intersecting with another ellipsoid
		 * @param other_obstacle the ellipsoid
		 * @return true if there is intersection
		 */
		bool is_intersecting_ellipsoid(const Ellipsoid& other_obstacle) const;

	public:
		/**
		 * @brief Constructor with name but empty state
		 * @param name name of the ellipsoid
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Ellipsoid(const std::string& name, double safety_margin=0);

		/**
		 * @brief Constructor with name and safety margin as an array
		 * @param name the obstacle name
		 * @param safety_margin the safety_margin
		 */
		explicit Ellipsoid(const std::string& name, const Eigen::Array3d& safety_margin);

		/**
		 * @brief Copy constructor from another ellipsoid
		 * @param ellipsoid the ellipsoid to copy
		 */
		explicit Ellipsoid(const Ellipsoid& ellipsoid);

		/**
		 * @brief Constructor with a name and center position as double values
		 * @param name the name of the obstacle
		 * @param cx the x coordinate of the center
		 * @param cy the y coordinate of the center
		 * @param cz the z coordinate of the center
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Ellipsoid(const std::string& name, double cx, double cy, double cz, double safety_margin=0);

		/**
		 * @brief Constructor with a name and center position as double values
		 * @param name the name of the obstacle
		 * @param cx the x coordinate of the center
		 * @param cy the y coordinate of the center
		 * @param cz the z coordinate of the center
		 * @param safety_margin the safety_margin as an array
		 */
		explicit Ellipsoid(const std::string& name, double cx, double cy, double cz, const Eigen::Array3d& safety_margin);

		/**
		 * @brief Constructor with a name and a state
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Ellipsoid(const std::string& name, const State& state, double safety_margin=0);

		/**
		 * @brief Constructor with a name, a state and the safety margin array
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param safety_margin the safety_margin array
		 */
		explicit Ellipsoid(const std::string& name, const State& state, const Eigen::Array3d& safety_margin);

		/**
		 * @brief Constructor with a name, a state and the reference position
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param reference_position the reference position
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Ellipsoid(const std::string& name, const State& state, const Eigen::Vector3d& reference_position, double safety_margin=0);

		/**
		 * @brief Constructor with a name, a state, the reference position and the safety margin array
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param reference_position the reference position
		 * @param safety_margin the safety_margin array
		 */
		explicit Ellipsoid(const std::string& name, const State& state, const Eigen::Vector3d& reference_position, const Eigen::Array3d& safety_margin);
		
		/**
		 * @brief Destructor
		 */
		~Ellipsoid();

		/**
		 * @brief Getter of the axis lengths
		 * @return the axis lengths
		 */
		const Eigen::Array3d& get_axis_lengths() const;

		/**
		 * @brief Getter of the axis length in one direction
		 * @param index the index of the length (0 for x, 1 for y and 2 for z)
		 * @return the length in the desired direction
		 */
		double get_axis_lengths(unsigned int index) const; 

		/**
		 * @brief Getter of the curvature factors
		 * @return the curvature factors
		 */
		const Eigen::Array3d& get_curvature_factor() const; 

		/**
		 * @brief Getter of the curvature factor in one direction
		 * @param index the index of the curvature factor (0 for x, 1 for y and 2 for z)
		 * @return the curvature factor in the desired direction
		 */
		double get_curvature_factor(unsigned int index) const ;

		/**
		 * @brief Setter of the axis lengths
		 * @param axis_lengths the new values
		 */
		void set_axis_lengths(const Eigen::Array3d& axis_lengths);

		/**
		 * @brief Setter of the curvature factors
		 * @param curvature_factor the new values
		 */
		void set_curvature_factor(const Eigen::Array3d& curvature_factor);

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
		 * @brief Overload of the print function
		 * @param os the current stream
		 * @return the strem appended with the obstacle description
		 */
		std::ostream& print(std::ostream& os) const override;

		/**
		 * @brief Compute the area of the ellipsoid
		 * @param is_include_safety_margin if true include the safety margin
		 * @return the area value
		 */
		double area(bool is_include_safety_margin=true) const;

		/**
		 * @brief Function to sample an obstacle from its parameterization
		 * @param nb_samples the number of sample points to generate 
		 * @param is_include_safety_margin if true should include the safety margin in computation
		 * @return the matrix of sample points
		 */
		Eigen::MatrixXd sample_from_parameterization(unsigned int nb_samples, bool is_include_safety_margin) const;

		/**
		 * @brief Function to check if a point is inside the obstacle
		 * @param point the point
		 * @return true if the point is inside
		 */
		bool point_is_inside(const Eigen::Vector3d& point) const;

		/**
		 * @brief Function to compute the repulsion vector when the agent is inside the obstacle
		 * @param agent the agent
		 * @return the repulsion vector
		 */
		Eigen::Vector3d compute_repulsion_vector(const Agent& agent) const;

		std::pair<bool, std::pair<Eigen::Vector3d, Eigen::Vector3d>> compute_interesection_points(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
	};

	inline const Eigen::Array3d& Ellipsoid::get_axis_lengths() const 
	{ 
		return this->axis_lengths;
	}

	inline double Ellipsoid::get_axis_lengths(unsigned int index) const 
	{ 
		return this->axis_lengths(index);
	}

	inline const Eigen::Array3d& Ellipsoid::get_curvature_factor() const 
	{ 
		return this->curvature_factor;
	}

	inline double Ellipsoid::get_curvature_factor(unsigned int index) const 
	{ 
		return this->curvature_factor(index);
	}

	inline void Ellipsoid::set_axis_lengths(const Eigen::Array3d& axis_lengths)
	{
		this->axis_lengths = axis_lengths;
	}

	inline void Ellipsoid::set_curvature_factor(const Eigen::Array3d& curvature_factor)
	{
		this->curvature_factor = curvature_factor;
	}

	inline std::ostream& Ellipsoid::print(std::ostream& os) const
	{ 
		os << static_cast<Obstacle>(*this) << std::endl;
		os << "axis lengths: (" << this->axis_lengths(0) << ", ";
		os << this->axis_lengths(1) << ", ";
		os << this->axis_lengths(2) << ")" << std::endl;
		os << "curvature factor: (" << this->curvature_factor(0) << ", ";
		os << this->curvature_factor(1) << ", ";
		os << this->curvature_factor(2) << ")";
			return os;
	}
}
#endif
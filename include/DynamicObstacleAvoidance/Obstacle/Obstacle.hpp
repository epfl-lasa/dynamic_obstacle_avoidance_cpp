/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_OBSTACLE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_OBSTACLE_H_

#include <eigen3/Eigen/Core>
#include <memory>
#include <deque>
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/matplotlibcpp.hpp"
#include <iostream>

namespace plt = matplotlibcpp;

namespace DynamicObstacleAvoidance
{
	class Ellipsoid;

	class Aggregate;

	class Agent;

	class Obstacle 
	{
	private:
		std::string name;
		State state;
		Eigen::Vector3d reference_position;
		std::string type;

		double safety_margin;

	protected:
		inline void set_type(const std::string& type)
		{
			this->type = type;
		}

		virtual bool is_intersecting_ellipsoid(const Ellipsoid& other_obstacle) const;

		bool is_intersecting_aggregate(const Aggregate& other_obstacle) const;

	public:
		explicit Obstacle(const std::string& name="obstacle", double safety_margin=0);
		explicit Obstacle(const std::string& name, double cx, double cy, double cz, double safety_margin=0);
		explicit Obstacle(const std::string& name, const State& state, double safety_margin=0);
		explicit Obstacle(const std::string& name, const State& state, const Eigen::Vector3d& reference_position, double safety_margin=0);
		~Obstacle();

		inline const std::string get_type() const 
		{ 
			return this->type;
		}

		inline std::string get_name() const
		{
			return this->name;
		}

		inline const State get_state() const 
		{ 
			return this->state;
		}

		inline const Pose get_pose() const 
		{ 
			return this->state.get_pose();
		}

		inline const Eigen::Vector3d get_position() const 
		{ 
			return this->state.get_position();
		}

		inline const Eigen::Quaterniond get_orientation() const 
		{ 
			return this->state.get_orientation();
		}

		inline const Eigen::Vector3d get_linear_velocity() const
		{
			return this->state.get_linear_velocity();
		}

		inline const Eigen::Vector3d get_angular_velocity() const
		{
			return this->state.get_angular_velocity();
		}

		inline const Eigen::Vector3d get_reference_position() const
		{
			return this->reference_position;
		}

		inline double get_safety_margin() const
		{ 
			return this->safety_margin;
		}

		inline void set_name(const std::string& name)
		{
			this->name = name;
		}

		inline void set_pose(const Pose& pose)
		{
			this->state.set_pose(pose);
		}

		inline void set_position(const Eigen::Vector3d& position)
		{
			this->state.set_position(position);
		}

		inline void set_position(double x, double y, double z)
		{
			this->state.set_position(x, y, z);
		}

		inline void set_orientation(const Eigen::Quaterniond& orientation)
		{
			this->state.set_orientation(orientation);
		}

		inline void set_linear_velocity(const Eigen::Vector3d& linear_velocity)
		{
			this->state.set_linear_velocity(linear_velocity);
		}

		inline void set_angular_velocity(const Eigen::Vector3d& angular_velocity)
		{
			this->state.set_angular_velocity(angular_velocity);
		}

		inline void set_reference_position(const Eigen::Vector3d& reference_position)
		{
			this->reference_position = reference_position;
		}

		inline friend std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle) 
		{ 
			return obstacle.print(os);
		}

		virtual inline std::ostream& print(std::ostream& os) const
		{
			os << this->type << " " << this->name << std::endl;
			os << this->state << std::endl;
			os << "reference position: (" << this->reference_position(0) << ", ";
			os << this->reference_position(1) << ", ";
			os << this->reference_position(2) << ")" << std::endl;
			os << "safety margin: " <<this->safety_margin;
	  		return os;
		}
		
		virtual Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;
		
		virtual double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin=0.) const;

		double compute_distance_to_agent(const Agent& agent) const;

		virtual void draw(const std::string& color="k", bool is3D=false) const;

		bool is_intersecting(const Obstacle& other_obstacle) const;

		bool is_intersecting(const std::deque<std::shared_ptr<Obstacle> >& other_obstacles) const;

		double get_repulsion_factor(const Agent& agent, double factor=2) const;

		virtual Eigen::MatrixXd sample_from_parameterization(unsigned int nb_samples, bool is_include_safety_margin) const;

		virtual bool point_is_inside(const Eigen::Vector3d& point) const;

		virtual Eigen::Vector3d compute_repulsion_vector(const Agent& agent) const;

		Eigen::Vector3d generate_repulsion(const Agent& agent, double repulsion_threshold=1.1) const;
	};
}
#endif
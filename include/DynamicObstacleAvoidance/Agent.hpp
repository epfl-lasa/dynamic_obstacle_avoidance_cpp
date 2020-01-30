/**
 * @class Modulation
 * @brief Class to define Modulations of the dynamical system
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_AGENT_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_AGENT_H_

#include <eigen3/Eigen/Core>
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"

namespace DynamicObstacleAvoidance
{
	class Agent
	{
	private:
		State state; ///< state (pose & velocities) of the agent
		double safety_margin; ///< safety margin represented as a circle around the agent
		std::shared_ptr<Obstacle> envelope; ///< envelope around and in front of the agent, used for turning on or off the avoidance
		double delta_t; //< delta time used to compute the feedforward of the envelope

		/**
		 * @brief Function to update the position and size of the envelope
		 */
		void update_envelope();

	public:
		/**
		 * @brief Default constructor
		 * @param safety_margin safety margin value
		 */
		explicit Agent(const double& safety_margin=0.0);

		/**
		 * @brief Constructor with provided State
		 * @param state the position and orientation of the agent
		 * @param safety_margin safety margin value
		 */
		explicit Agent(const State& state, const double& safety_margin=0.0);

		/**
		 * @brief Copy constructor
		 */
		Agent(const Agent& agent);

		/**
		 * @brief Getter of the envelope
		 * @return the envelope
		 */
		inline const Obstacle& get_envelope() const 
		{ 
			return *(this->envelope);
		}

		/**
		 * @brief Getter of the state
		 * @return the state of the agent
		 */
		inline const State get_state() const 
		{ 
			return this->state;
		}

		/**
		 * @brief Getter of the pose of the agent from its state
		 * @return the pose of the agent
		 */
		inline const Pose get_pose() const 
		{ 
			return this->state.get_pose();
		}

		/**
		 * @brief Getter of the position of the agent from its state
		 * @return the position of the agent
		 */
		inline const Eigen::Vector3d get_position() const 
		{ 
			return this->state.get_position();
		}

		/**
		 * @brief Getter of the orientation of the agent from its state
		 * @return the orientation of the agent
		 */
		inline const Eigen::Quaterniond get_orientation() const 
		{ 
			return this->state.get_orientation();
		}

		/**
		 * @brief Getter of the linear velocity of the agent from its state
		 * @return the linear velocity of the agent
		 */
		inline const Eigen::Vector3d get_linear_velocity() const
		{
			return this->state.get_linear_velocity();
		}

		/**
		 * @brief Getter of the angular velocity of the agent from its state
		 * @return the angular velocity
		 */
		inline const Eigen::Vector3d get_angular_velocity() const
		{
			return this->state.get_angular_velocity();
		}

		/**
		 * @brief Getter of the safety margin 
		 * @return the safety margin
		 */
		inline double get_safety_margin() const
		{ 
			return this->safety_margin;
		}

		/**
		 * @brief Setter of the safety margin
		 * @param safety_margin the new safety margin
		 */
		inline void set_safety_margin(const double& safety_margin)
		{ 
			this->safety_margin = safety_margin;
		}

		/**
		 * @brief Setter of the pose of the agent
		 * @param pose the new pose
		 */
		inline void set_pose(const Pose& pose)
		{
			this->state.set_pose(pose);
			this->update_envelope();
		}

		/**
		 * @brief Setter of the position of the agent
		 * @param position the new position
		 */
		inline void set_position(const Eigen::Vector3d& position)
		{
			this->state.set_position(position);
			this->update_envelope();
		}

		/**
		 * @brief Setter of the orientation of the agent
		 * @param orientation the new orientation
		 */
		inline void set_orientation(const Eigen::Quaterniond& orientation)
		{
			this->state.set_orientation(orientation);
			this->update_envelope();
		}

		/**
		 * @brief Setter of the linear velocity of the agent
		 * @param linear_velocity the new linear velocity
		 */
		inline void set_linear_velocity(const Eigen::Vector3d& linear_velocity)
		{
			this->state.set_linear_velocity(linear_velocity);
			this->update_envelope();
		}

		/**
		 * @brief Setter of the angular velocity of the agent
		 * @param angular_velocity the new angular velocity
		 */
		inline void set_angular_velocity(const Eigen::Vector3d& angular_velocity)
		{
			this->state.set_angular_velocity(angular_velocity);
			this->update_envelope();
		}

		/**
		 * @brief Function to apply a transformation to the pose of the agent
		 * @param p the transformation to apply
		 */
		inline void transform(const Pose& p)
		{
			this->set_pose(p * this->get_pose());
		}

		/**
		 * @brief Function to compute an agent at the transformed location
		 * @param p the transformation to apply
		 * @return the agent at the desired location
		 */
		inline const Agent transformed(const Pose& p) const
		{
			Agent agent(*this);
			agent.transform(p);
			return agent;
		}

		/**
		 * @brief operator<< overloading to print an agent in console
		 */
		inline friend std::ostream& operator<<(std::ostream& os, const Agent& agent) 
		{ 
			os << "Agent" << std::endl;
	  		os << agent.state << std::endl;
	  		os << "safety margin: " << agent.safety_margin;
	  		return os;
		}

		/**
		 * @brief Indicate if the agent is in the provided obstacle
		 * @param obstacle the obstacle
		 * @return true if the agent is in the obstacle
		 */
		bool in_obstacle(const Obstacle& obstacle) const;

		/**
		 * @brief Indicate if there exist a free path to the arget 
		 * @param target the location to reach
		 * @param obstacles the list of obstacles in the environment
		 * @return true if a free path exists
		 */
		bool exist_path(const Eigen::Vector3d& target, const std::deque<std::shared_ptr<Obstacle> >& obstacles) const;
	};
}
#endif
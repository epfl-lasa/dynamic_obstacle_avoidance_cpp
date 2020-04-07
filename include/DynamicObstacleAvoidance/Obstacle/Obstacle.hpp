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
		std::string name; //< name of the obstacle
		State state; //< state of the obstacle (pose + velocities)
		Eigen::Vector3d reference_position; //< reference position expressed in the same frame than the obstacle
		std::string type; //< type of obstacle
		Eigen::Array3d safety_margin; //< array of safety margins in x,y,z directions

	protected:
		/**
		 * @brief Setter for the type attribute
		 * @param type the new type
		 */
		void set_type(const std::string& type);

		/**
		 * @brief Function to check if the obstacle is intersecting with another ellipsoid
		 * @param other_obstacle the ellipsoid
		 * @return true if there is intersection
		 */
		virtual bool is_intersecting_ellipsoid(const Ellipsoid& other_obstacle) const;

		/**
		 * @brief Function to check if the obstacle is intersecting with another aggregate of obstacles
		 * @param other_obstacle the aggregate of obstacles
		 * @return true if there is intersection
		 */
		bool is_intersecting_aggregate(const Aggregate& other_obstacle) const;

	public:
		/**
		 * @brief Empty constructor with default name and safety margin
		 * @param name the obstacle name (default="obstacle")
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Obstacle(const std::string& name="obstacle", double safety_margin=0);

		/**
		 * @brief Constructor with name and safety margin as an array
		 * @param name the obstacle name
		 * @param safety_margin the safety_margin
		 */
		explicit Obstacle(const std::string& name, const Eigen::Array3d& safety_margin);

		/**
		 * @brief Constructor with a name and center position as double values
		 * @param name the name of the obstacle
		 * @param cx the x coordinate of the center
		 * @param cy the y coordinate of the center
		 * @param cz the z coordinate of the center
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Obstacle(const std::string& name, double cx, double cy, double cz, double safety_margin=0);
		
		/**
		 * @brief Constructor with a name and center position as double values
		 * @param name the name of the obstacle
		 * @param cx the x coordinate of the center
		 * @param cy the y coordinate of the center
		 * @param cz the z coordinate of the center
		 * @param safety_margin the safety_margin as an array
		 */
		explicit Obstacle(const std::string& name, double cx, double cy, double cz, const Eigen::Array3d& safety_margin);
		
		/**
		 * @brief Constructor with a name and a state
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Obstacle(const std::string& name, const State& state, double safety_margin=0);
		
		/**
		 * @brief Constructor with a name, a state and the safety margin array
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param safety_margin the safety_margin array
		 */
		explicit Obstacle(const std::string& name, const State& state, const Eigen::Array3d& safety_margin);
		
		/**
		 * @brief Constructor with a name, a state and the reference position
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param reference_position the reference position
		 * @param safety_margin the safety_margin (default=0 in all axes)
		 */
		explicit Obstacle(const std::string& name, const State& state, const Eigen::Vector3d& reference_position, double safety_margin=0);
		
		/**
		 * @brief Constructor with a name, a state, the reference position and the safety margin array
		 * @param name the name of the obstacle
		 * @param state the state (position, orientation and velocities)
		 * @param reference_position the reference position
		 * @param safety_margin the safety_margin array
		 */
		explicit Obstacle(const std::string& name, const State& state, const Eigen::Vector3d& reference_position, const Eigen::Array3d& safety_margin);
		
		/**
		 * @brief Destructor
		 */
		~Obstacle();

		/**
		 * @brief Getter of the type as const value
		 * @return the type
		 */
		const std::string& get_type() const;

		/**
		 * @brief Getter of the type as non const value
		 * @return the type
		 */
		const std::string& get_name() const;

		/**
		 * @brief Getter of the State
		 * @return the state
		 */
		const State& get_state() const;

		/**
		 * @brief Getter of the Pose
		 * @return the pose
		 */
		const Pose& get_pose() const;

		/**
		 * @brief Getter of the position
		 * @return the position
		 */
		const Eigen::Vector3d& get_position() const;

		/**
		 * @brief Getter of the orientation
		 * @return the orientation
		 */
		const Eigen::Quaterniond& get_orientation() const;

		/**
		 * @brief Getter of the linear velocity
		 * @return the linear velocity
		 */
		const Eigen::Vector3d& get_linear_velocity() const;

		/**
		 * @brief Getter of the angular velocity
		 * @return the angular velocity
		 */
		const Eigen::Vector3d& get_angular_velocity() const;

		/**
		 * @brief Getter of the reference position
		 * @return the reference position
		 */
		const Eigen::Vector3d& get_reference_position() const;

		/**
		 * @brief Getter of the safety margin
		 * @return the safety margin
		 */
		const Eigen::Array3d& get_safety_margin() const;

		/**
		 * @brief Setter of the name
		 * @param name the name
		 */
		void set_name(const std::string& name);

		/**
		 * @brief Setter of the pose
		 * @param pose the pose
		 */
		void set_pose(const Pose& pose);

		/**
		 * @brief Setter of the position
		 * @param position the position
		 */
		void set_position(const Eigen::Vector3d& position);

		/**
		 * @brief Setter of the position with three double values
		 * @param x the x position
		 * @param y the y position
		 * @param z the z position
		 */
		void set_position(double x, double y, double z);

		/**
		 * @brief Setter of the orientation
		 * @param orientation the orientation
		 */
		void set_orientation(const Eigen::Quaterniond& orientation);

		/**
		 * @brief Setter of the linear velocity
		 * @param linear_velocity the linear velocity
		 */
		void set_linear_velocity(const Eigen::Vector3d& linear_velocity);

		/**
		 * @brief Setter of the angular velocity
		 * @param angular_velocity the angular velocity
		 */
		void set_angular_velocity(const Eigen::Vector3d& angular_velocity);

		/**
		 * @brief Setter of the reference position
		 * @param reference_position the reference position
		 */
		virtual void set_reference_position(const Eigen::Vector3d& reference_position);

		/**
		 * @brief Overload of the print function
		 * @param os the current stream
		 * @return the strem appended with the obstacle description
		 */
		virtual std::ostream& print(std::ostream& os) const;

		/**
		 * @brief Overload of the << operator
		 * @param os the current stream
		 * @param obstacle the obstacle to print
		 * @return the strem appended with the obstacle description
		 */
		friend std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle) ;
		
		/**
		 * @brief Function to compute the normal on the surface to the agent wrt the reference point 
		 * @param agent the agent 
		 * @return the normal vector
		 */
		virtual Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		/**
		 * @brief Function to compute the distance between a point and the surface of the obstacle wrt the reference point 
		 * @param point the external point
		 * @param safety_margin the safety margin to add
		 * @return the distance value (1 if the point is on the surface)
		 */
		virtual double compute_distance_to_point(const Eigen::Vector3d& point, const Eigen::Array3d& safety_margin) const;

		/**
		 * @brief Function to compute the distance between a point and the surface of the obstacle wrt the reference point 
		 * @param point the external point
		 * @param safety_margin the safety margin to add (default is 0 in all directions)
		 * @return the distance value (1 if the point is on the surface)
		 */
		double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin=0.) const;

		/**
		 * @brief Function to compute the distance to an agent wrt the safety margin 
		 * @param agent the agent
		 * @return the distance value (1 if the agent is on the surface)
		 */
		double compute_distance_to_agent(const Agent& agent) const;

		/**
		 * @brief Function to draw an agent
		 * @param color the color to apply
		 * @param axis axis to draw (default = "xy")
		 */
		virtual void draw(const std::string& color="k", const std::string& axis="xy") const;

		/**
		 * @brief Function to check if the obstacle is intersecting with another obstacle
		 * @param other_obstacle the other obstacle
		 * @return true if there is intersection
		 */
		bool is_intersecting(const Obstacle& other_obstacle) const;

		/**
		 * @brief Function to check if the obstacle is intersecting with another obstacle
		 * @param other_obstacle the other obstacle as a shared_ptr
		 * @return true if there is intersection
		 */
		bool is_intersecting(const std::deque<std::shared_ptr<Obstacle> >& other_obstacles) const;

		/**
		 * @brief Function to compute the repulsion factor when the agent is inside the obstacle
		 * @param agent the agent
		 * @param factor multiplication factor 
		 * @return the repulsion factor
		 */
		double get_repulsion_factor(const Agent& agent, double factor=2) const;

		/**
		 * @brief Function to sample an obstacle from its parameterization
		 * @param nb_samples the number of sample points to generate 
		 * @param is_include_safety_margin if true should include the safety margin in computation
		 * @return the matrix of sample points
		 */
		virtual Eigen::MatrixXd sample_from_parameterization(unsigned int nb_samples, bool is_include_safety_margin) const;

		/**
		 * @brief Function to check if a point is inside the obstacle
		 * @param point the point
		 * @return true if the point is inside
		 */
		virtual bool point_is_inside(const Eigen::Vector3d& point) const;

		/**
		 * @brief Function to compute the repulsion vector when the agent is inside the obstacle
		 * @param agent the agent
		 * @return the repulsion vector
		 */
		virtual Eigen::Vector3d compute_repulsion_vector(const Agent& agent) const;

		/**
		 * Function to generate a repulsion when the agent is inside the obstacle
		 * @param agent the agent
		 * @param repulsion_threshold the gamma distance under which a repulsion should be generated
		 * @return the repulsion velocity
		 */
		Eigen::Vector3d generate_repulsion(const Agent& agent, double repulsion_threshold=1.1) const;

		/**
		 * @brief Function to check if the aggregate is closed, i.e. every surface point is farther than the min radius
		 * @return true if the aggregate is closed
		 */
		virtual bool is_closed() const;

		virtual std::pair<bool, std::pair<Eigen::Vector3d, Eigen::Vector3d>> compute_interesection_points(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);

		void shift_reference_point(const Agent& agent);
	};

	inline void Obstacle::set_type(const std::string& type)
	{
		this->type = type;
	}

	inline const std::string& Obstacle::get_type() const 
	{ 
		return this->type;
	}

	inline const std::string& Obstacle::get_name() const
	{
		return this->name;
	}

	inline const State& Obstacle::get_state() const 
	{ 
		return this->state;
	}

	inline const Pose& Obstacle::get_pose() const 
	{ 
		return this->state.get_pose();
	}

	inline const Eigen::Vector3d& Obstacle::get_position() const 
	{ 
		return this->state.get_position();
	}

	inline const Eigen::Quaterniond& Obstacle::get_orientation() const 
	{ 
		return this->state.get_orientation();
	}

	inline const Eigen::Vector3d& Obstacle::get_linear_velocity() const
	{
		return this->state.get_linear_velocity();
	}

	inline const Eigen::Vector3d& Obstacle::get_angular_velocity() const
	{
		return this->state.get_angular_velocity();
	}

	inline const Eigen::Vector3d& Obstacle::get_reference_position() const
	{
		return this->reference_position;
	}

	inline const Eigen::Array3d& Obstacle::get_safety_margin() const
	{ 
		return this->safety_margin;
	}

	inline void Obstacle::set_name(const std::string& name)
	{
		this->name = name;
	}

	inline void Obstacle::set_pose(const Pose& pose)
	{
		this->state.set_pose(pose);
	}

	inline void Obstacle::set_position(const Eigen::Vector3d& position)
	{
		this->state.set_position(position);
	}

	inline void Obstacle::set_position(double x, double y, double z)
	{
		this->state.set_position(x, y, z);
	}

	inline void Obstacle::set_orientation(const Eigen::Quaterniond& orientation)
	{
		this->state.set_orientation(orientation);
	}

	inline void Obstacle::set_linear_velocity(const Eigen::Vector3d& linear_velocity)
	{
		this->state.set_linear_velocity(linear_velocity);
	}

	inline void Obstacle::set_angular_velocity(const Eigen::Vector3d& angular_velocity)
	{
		this->state.set_angular_velocity(angular_velocity);
	}

	inline void Obstacle::set_reference_position(const Eigen::Vector3d& reference_position)
	{
		this->reference_position = reference_position;
	}

	inline std::ostream& Obstacle::print(std::ostream& os) const
	{
		os << this->type << " " << this->name << std::endl;
		os << this->state << std::endl;
		os << "reference position: (" << this->reference_position(0) << ", ";
		os << this->reference_position(1) << ", ";
		os << this->reference_position(2) << ")" << std::endl;
		os << "safety margin: " << this->safety_margin.transpose();
  		return os;
	}

	inline std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle) 
	{ 
		return obstacle.print(os);
	}

	inline bool Obstacle::is_closed() const
	{
		return true;
	}
}
#endif
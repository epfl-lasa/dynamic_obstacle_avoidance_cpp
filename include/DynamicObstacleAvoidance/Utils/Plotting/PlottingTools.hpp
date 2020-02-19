#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_PLOTTING_PLOTTINGTOOLS_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_PLOTTING_PLOTTINGTOOLS_H_

#include <deque>
#include <vector>
#include <eigen3/Eigen/Core>
#include <memory>
#include <string>

#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/matplotlibcpp.hpp"

namespace plt = matplotlibcpp;

namespace DynamicObstacleAvoidance
{
	namespace PlottingTools 
	{
		/**
		 * @brief Draw the obstacles on the 2d plane defined by axis
		 * @param obstacles the list of obstacles to draw
		 * @param axis the axis ("xy" or "xz")
		 */
		void plot_obstacles(const std::deque<std::shared_ptr<Obstacle> >& obstacles, const std::string& axis);

		/**
		 * @brief Draw the agent in the 2d plane defined by axis
		 * @param agent the agent to draw
		 * @param axis the axis ("xy" or "xz")
		 * @param color the color to draw the agent with (default = blue)
		 */
		void plot_agent(const Agent& agent, const std::string& axis, const std::string& color="b");

		/**
		 * @brief Draw the agent and its trajectory on the 2d plane defined by axis
		 * @param goal the desired target
		 * @param position_history the trajectory points
		 * @param axis the axis ("xy" or "xz")
		 */
		void plot_agent_trajectory(const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& axis);

		/**
		 * @brief plot the configuration with agent and obstacles
		 * @param agent the agent
		 * @param obstacles the obstacles
		 * @param goal the desired target
		 * @param position_history the trajectory points
		 * @param savefile name of the file to save in tmp folder (if empty the image is not saved)
		 * @param is_show if true the image is shown as a pop-up
		 */
		void plot_configuration(const Agent& agent, const std::deque<std::shared_ptr<Obstacle> >& obstacles, const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& savefile="", bool is_show=true, bool is3D=false);
	
		/**
		 * @brief plot the configuration with only obstacles
		 * @param obstacles the obstacles
		 * @param savefile name of the file to save in tmp folder (if empty the image is not saved)
		 * @param is_show if true the image is shown as a pop-up
		 */
		void plot_configuration(const std::deque<std::shared_ptr<Obstacle> >& obstacles, const std::string& savefile="", bool is_show=true, bool is3D=false);
	
		/**
		 * @brief plot the configuration for the guard carrying scenario
		 * @param platform the platform
		 * @param guard the guard
		 * @param obstacles the obstacles
		 * @param goal the desired target
		 * @param position_history the trajectory points
		 * @param savefile name of the file to save in tmp folder (if empty the image is not saved)
		 * @param is_show if true the image is shown as a pop-up
		 */
		void plot_guard_configuration(const Agent& platform, const Agent& guard, const std::deque<std::shared_ptr<Obstacle> >& obstacles, const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& savefile, bool is_show, bool is3D=false);
	}
}

#endif
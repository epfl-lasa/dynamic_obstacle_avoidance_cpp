/**
 * @class Modulation
 * @brief Class to define Modulations of the dynamical system
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_MODULATION_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_MODULATION_H_

#include <deque>
#include <tuple>
#include <utility>
#include <functional>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <algorithm>
#include <memory>
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Environment.hpp"

namespace DynamicObstacleAvoidance
{
    namespace Modulation
    {
    	/**
    	 * @brief Function to weight the obstacles based on the distances to the agent
    	 * @param distances matrix of distances (one per obstacle) 
    	 * @param critical_distance critical distance at wich to consider the obstacle
    	 * @param weight_power power of the weight 
    	 * @return array of wieghts (one per obstacle)
    	 */
        Eigen::ArrayXd weight_obstacles(const Eigen::ArrayXd& distances, double critical_distance, double weight_power);

        /**
         * @brief Function to compute the basis matrices for the modulation
         * @param normal_vector normal vector to the agent on the surface of the obstacle wrt the reference point
         * @param agent the agent
         * @param obstacle the obstacle
         * @return the basis matrices 
         */
        std::pair<Eigen::Matrix3d, Eigen::Matrix3d> compute_basis_matrices(const Eigen::Vector3d& normal_vector, const Agent& agent, const Obstacle& obstacle);
        
        /**
         * @brief Fnction to compute the eigenvalues based on the distance
         * @param distance_to_obstacle the distance to the obstacle
         * @param reactivity_factor factor used when the point is inside the obstacle to increase the modulation
         */
        Eigen::DiagonalMatrix<double, 3> compute_diagonal_eigenvalues(double distance_to_obstacle, double reactivity_factor=1.0);
        
        /**
         * @brief Function to compute the relative velocities for each obstacles
         * @param agent the agent
         * @param obstacles the list of all obstacles
         * @return the vector of relative velocities (linear and angular)
         */
        std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_relative_velocities(const Agent& agent, const std::deque<std::shared_ptr<Obstacle> >& obstacles, const Eigen::ArrayXd& distances, const Eigen::ArrayXd& weights, const std::deque<Eigen::Matrix3d>& orthogonal_basis_list);
        
        /**
         * @brief Function to compute the modulation matrices
         * @param agent the agent
         * @param obstacle the obstacle
         * @return the modulation matrices
         */
        std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, double> compute_modulation_matrix(const Agent& agent, const Obstacle& obstacle);
        
        /**
         * @brief Modulate the velocity to avoid the obstacles
         * @param agent the agent 
         * @param environment the environment that contains the list of obstacles
         * @param is_local true to turn on the local modulation
         * @param add_repulsion true to add a repulsion when the agent is in the obstacle
         * @param critical_distance critical distance for repulsion (default = 1, i.e. the agent is on the obstacle)
         * @param weight_power power of the weight of the obstacles
         * @return the modulated velocity
         */
        Eigen::Vector3d modulate_velocity(const Agent& agent, const Environment& environment, bool is_local=false, bool add_repulsion=false, double critical_distance=1.0, double weight_power=2.0);
    }
}

#endif
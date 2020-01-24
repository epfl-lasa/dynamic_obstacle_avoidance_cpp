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
        Eigen::ArrayXd weight_obstacles(const Eigen::ArrayXd& distances, double critical_distance, double weight_power);

        std::pair<Eigen::Matrix3d, Eigen::Matrix3d> compute_basis_matrices(const Eigen::Vector3d& normal_vector, const Agent& agent, const Obstacle& obstacle);
        
        Eigen::DiagonalMatrix<double, 3> compute_diagonal_eigenvalues(double distance_to_obstacle, double reactivity_factor=1.0);
        
        std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_relative_velocities(const Agent& agent, const std::deque<std::shared_ptr<Obstacle> >& obstacles, const Eigen::ArrayXd& distances, const Eigen::ArrayXd& weights, const std::deque<Eigen::Matrix3d>& orthogonal_basis_list);
        
        std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, double> compute_modulation_matrix(const Agent& agent, const Obstacle& obstacle);
        
        Eigen::Vector3d modulate_velocity(const Agent& agent, const Environment& environment, bool is_local=false, bool add_repulsion=false, double critical_distance=1.0, double weight_power=2.0);
    }
}

#endif
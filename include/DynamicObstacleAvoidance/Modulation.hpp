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
#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"

namespace Modulation
{
    Eigen::ArrayXd weight_obstacles(const Eigen::ArrayXd& distances, const double& critical_distance, const double& weight_power);
    std::pair<Eigen::Matrix3d, Eigen::Matrix3d> compute_basis_matrices(const Eigen::Vector3d& normal_vector, const Agent& agent, const Obstacle& obstacle);
    Eigen::DiagonalMatrix<double, 3> compute_diagonal_eigenvalues(const double& distance_to_obstacle, const double& reactivity_factor=1.0);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_relative_velocities(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const Eigen::ArrayXd& distances, const Eigen::ArrayXd& weights, const std::deque<Eigen::Matrix3d>& orthogonal_basis_list);
    /**
    * @brief The function evaluates the gamma function and all necessary components needed to construct the modulation function, to ensure safe avoidance of the obstacles.
    * Beware that this function is constructed for ellipsoid only, but the algorithm is applicable to star shapes.
    * @param[in] robotPos Position of the robot in the obstacle reference frame
    * @param[in] obstacle Description of the obstacle with parameters
    * @param[in] rotation Rotation matrix
    * @param[out] basis Basis matrix with rows the reference and tangent to the obstacles surface
    * @param[out] basisOrthogonal Orthogonal basis matrix with rows the normal and tangent
    * @param[out] eigenValues Eigenvalue matrix which is responsible for the modulation
    */
    std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, double> compute_modulation_matrix(const Agent& agent, const Obstacle& obstacle);
    Eigen::Vector3d modulate_velocity(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const double& critical_distance=1.0, const double& weight_power=2.0);
}

#endif
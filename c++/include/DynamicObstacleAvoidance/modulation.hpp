/**
 * @class Modulation
 * @brief Class to define Modulations of the dynamical system
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_MODULATION_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_MODULATION_H_

#include <list>
#include <eigen3/Eigen/Core>
#include "DynamicObstacleAvoidance/obstacle.hpp"
#include "DynamicObstacleAvoidance/agent.hpp"

namespace Modulation
{
     Eigen::VectorXf compute_direction_to_obstacle(const Obstacle& obstacle, const Eigen::VectorXf& agent_position);
     Eigen::VectorXf compute_normal_to_obstacle(const Obstacle& obstacle, const Eigen::VectorXf& agent_position);
     double compute_distance_to_obstacle(const Obstacle& obstacle, const Eigen::VectorXf& agent_position);
     Eigen::MatrixXf compute_eigenvalues();
     Eigen::VectorXf weight_obstacles(Eigen::VectorXf& distances, double critical_distance, double weight_power);


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
     double compute_modulation_matrix(const Agent& agent, const Obstacle& obstacle, Eigen::MatrixXf& basis, Eigen::MatrixXf& orthogonal_basis, Eigen::MatrixXf& eigenvalues);
	Eigen::VectorXf modulate_velocity(Eigen::VectorXf& agent_pose, Eigen::VectorXf& agent_velocity, const std::list<Obstacle>& obstacles, const std::list<Eigen::VectorXf>& attractors_position);
}

#endif
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

using namespace Eigen;
using namespace std;

namespace modulation 
{
	/**
	 * @brief The function evaluates the gamma function and all necessary components needed to construct the modulation function, to ensure safe avoidance of the obstacles.
     * Beware that this function is constructed for ellipsoid only, but the algorithm is applicable to star shapes.
     * @param[in] robotPos Position of the robot in the obstacle reference frame
     * @param[in] obstacle Description of the obstacle with parameters
     * @param[in] rotation Rotation matrix
     * @param[out] basis Basis matrix with rows the reference and tangent to the obstacles surface
     * @param[out] basisOrthogonal Orthogonal basis matrix with rows the normal and tangent
     * @param[out] eigenValues Eigenvalue matrix which is responsible for the modulation
     * @param[out] gamma Distance function to the obstacle surface (in direction of the reference vector)
	 */
	void compute_modulation_matrix(const VectorXf& robotPos, const Obstacle& obstacle, const MatrixXf& rotation, MatrixXf& basis, MatrixXf& basisOrthogonal, MatrixXf& eigenValues, double gamma);

	void modulate_dynamical_system(const VectorXf& robotPos, const list<Obstacle>& obstacles, const list<VectorXf>& attractorsPos, double weightPow, VectorXf& xD);
}

#endif
#include "DynamicObstacleAvoidance/modulation.hpp"

void compute_modulation_matrix(const VectorXf& robotPos, const Obstacle& obstacle, const MatrixXf& rotation, MatrixXf& basis, MatrixXf& basisOrthogonal, MatrixXf& eigenValues, double& gamma)
{
	VectorXf axis = obstacle.get_safety_margin() * obstacle.get_axis();
	VectorXf curvature = obstacle.get_curvature();

	// compute gamma and normal vector based on curvature for ellipsoid
	VectorXf normal(robotPos.size());
	for(int i=0; i<robotPos.size(); ++i) 
	{
		double tmp = robotPos(i) / axis(i);
		gamma += pow(tmp, 2 * curvature(i));
		normal(i) = pow(2 * curvature(i) / axis(i) * tmp, 2 * curvature(i) - 1);
	}
	// normalize the normal vector
	normal /= normal.norm(); 
}
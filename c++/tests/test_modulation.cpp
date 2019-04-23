#include "DynamicObstacleAvoidance/modulation.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>

TEST(ComputeModulationMatrixTest, PositiveNos)
{
	Eigen::VectorXf robotPos(2);
	robotPos << 0., 1.;

	Eigen::VectorXf obsPos(2);
	obsPos << 0.5, 0.5;

	Eigen::VectorXf obsRot(1);
	obsRot << 0.1;

	Eigen::VectorXf obsAxis(2);
	obsPos << 1., 0.;

	Eigen::VectorXf obsCurv(2);
	obsPos << 0.1, 0.1;

	double margin = 0.3;
	
	//Obstacle obstacle(obsPos, obsRot, margin);

	Eigen::MatrixXf rotation;
	Eigen::MatrixXf basis;
	Eigen::MatrixXf basisOrthogonal;
	Eigen::MatrixXf eigenValues;
	double gamma;
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
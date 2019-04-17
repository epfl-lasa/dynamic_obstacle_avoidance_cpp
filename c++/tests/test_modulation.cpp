#include "DynamicObstacleAvoidance/modulation.hpp"
#include <gtest/gtest.h>

TEST(ComputeModulationMatrixTest, PositiveNos)
{
	VectorXf robotPos(2);
	robotPos << 0., 1.;

	VectorXf obsPos(2);
	obsPos << 0.5, 0.5;

	VectorXf obsRot(1);
	obsRot << 0.1;

	VectorXf obsAxis(2);
	obsPos << 1., 0.;

	VectorXf obsCurv(2);
	obsPos << 0.1, 0.1;

	double margin = 0.3;
	
	Obstacle obstacle(obsPos, obsRot, obsAxis, obsCurv, margin);

	MatrixXf rotation;
	MatrixXf basis;
	MatrixXf basisOrthogonal;
	MatrixXf eigenValues;
	double gamma;
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
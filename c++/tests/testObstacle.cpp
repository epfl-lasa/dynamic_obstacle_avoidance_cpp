#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>

TEST(ComputeDistanceToExternalPointNullPosition, PositiveNos)
{
	Eigen::Vector3f position;
	position << 0, 0, 0;

	Eigen::Vector4f orientation;
	position << 0, 0, 0, 1;

	Ellipsoid e(position, orientation);

	Eigen::Vector3f agent_position;
	agent_position << 1, 0, 0;
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
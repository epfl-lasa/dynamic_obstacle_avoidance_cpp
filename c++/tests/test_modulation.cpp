#include "DynamicObstacleAvoidance/modulation.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>

TEST(WeightObstacleTestUnderCriticalDistance, PositiveNos)
{
	Eigen::ArrayXf distances(5);
	distances << 0, 1, 2, 3, 4;
	double critical_distance = 1.0;
	double weight_power = 2;
	Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);

	Eigen::ArrayXf true_values(5);
	true_values << 0.5, 0.5, 0. , 0. , 0. ;

	std::cerr << "weights: " << weights << std::endl;
	std::cerr << "-------" << std::endl;
	std::cerr << "truth: " << true_values << std::endl;

	for(int i=0; i<weights.size(); ++i) ASSERT_NEAR(weights(i), true_values(i), 0.00001);
}

TEST(WeightObstacleTestFirstEqualCriticalDistance, PositiveNos)
{
	Eigen::ArrayXf distances(5);
	distances << 1, 2, 3, 4, 5;
	double critical_distance = 1.0;
	double weight_power = 2;
	Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);

	Eigen::ArrayXf true_values(5);
	true_values << 1., 0., 0., 0., 0.;

	std::cerr << "weights: " << weights << std::endl;
	std::cerr << "-------" << std::endl;
	std::cerr << "truth: " << true_values << std::endl;

	for(int i=0; i<weights.size(); ++i) ASSERT_NEAR(weights(i), true_values(i), 0.00001);
}

TEST(WeightObstacleTestAboveCriticalDistance, PositiveNos)
{
	Eigen::ArrayXf distances(5);
	distances << 2, 3, 4, 5, 6;
	double critical_distance = 1.0;
	double weight_power = 2;
	Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);

	Eigen::ArrayXf true_values(5);
	true_values << 0.6832416 , 0.1708104 , 0.07591573, 0.0427026 , 0.02732966;

	std::cerr << "weights: " << weights << std::endl;
	std::cerr << "-------" << std::endl;
	std::cerr << "truth: " << true_values << std::endl;

	for(int i=0; i<weights.size(); ++i) ASSERT_NEAR(weights(i), true_values(i), 0.00001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
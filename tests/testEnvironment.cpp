#include "DynamicObstacleAvoidance/Environment.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>

using namespace DynamicObstacleAvoidance;

TEST(AddObstacle, PositiveNos)
{
	Environment env;

	Eigen::Vector3d pos(MathTools::rand_float(3,-3), MathTools::rand_float(3,-3), 0);
	Eigen::Quaterniond rot(Eigen::AngleAxisd(MathTools::rand_float(2)*M_PI, Eigen::Vector3d::UnitZ()));
	auto ptrE = std::make_shared<Ellipsoid>("e1", State(pos, rot));
	ptrE->set_axis_lengths(Eigen::Array3d(MathTools::rand_float(3,0.2), MathTools::rand_float(3,0.2), 0));

	env.add_obstacle(ptrE);

	EXPECT_TRUE(env["e1"]->get_position()(0) == pos(0));
}

TEST(GetList, PositiveNos)
{
	unsigned int nb_obstacles = 3;
	Environment env(false);
	for(unsigned int i=0; i<nb_obstacles; ++i)
	{
		auto ptrE = std::make_shared<Ellipsoid>("e" + std::to_string(i));
		env.add_obstacle(ptrE);
	}

	auto obstacle_list = env.get_obstacle_list();
	EXPECT_TRUE(obstacle_list.size() == nb_obstacles);

	std::cout << *obstacle_list[0] << std::endl;
}

TEST(Aggregation, PositiveNos)
{
	Environment env;
	auto e1 = std::make_shared<Ellipsoid>("e1", 0.5, 0, 0, 0);
	auto e2 = std::make_shared<Ellipsoid>("e2", -0.5, 0, 0, 0);
	env.add_obstacle(e1);
	env.add_obstacle(e2);

	auto obstacle_list = env.get_obstacle_list();
	EXPECT_TRUE(obstacle_list.size() == 1);
	//PlottingTools::plot_configuration(obstacle_list, "test_aggregate1", true);

	e1->set_position(Eigen::Vector3d(2,0,0));
	env.update();
	
	obstacle_list = env.get_obstacle_list();
	EXPECT_TRUE(obstacle_list.size() == 2);
	//PlottingTools::plot_configuration(obstacle_list, "test_aggregate2", true);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
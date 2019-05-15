#include "DynamicObstacleAvoidance/Obstacle/PointCloudToObstacle.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <iostream>

TEST(TestFitEllipsoid, PositiveNos)
{
	Eigen::MatrixXd surface_points(3,10);
	surface_points << 0.68026725, 3.80951844, -1.6613724, -0.25730725, 0.69801745, -5.8991695, -5.46727359, -5.68929574, -4.95414465, -5.78085261,
					  -0.01634235, 0.79848348, -0.57766695, -0.15555173, 0.17456129, 4.01799145, 3.08517251, 3.23357203, 2.85638922, 2.40213298,
					  0,0,0,0,0,0,0,0,0,0;

	PointCloudToObstacle pco;
	std::deque<Ellipsoid> ellipse_list = pco.fit_ellipsoids(surface_points, 2);
	ASSERT_EQ(ellipse_list.size(), 2);

	for(Ellipsoid e:ellipse_list)
	{
		std::cerr << "position: " << e.get_position() << std::endl;
		std::cerr << "rotation: " << e.get_orientation().w() << ", " << e.get_orientation().x() << ", " << e.get_orientation().y() << ", " << e.get_orientation().z() << std::endl;
		std::cerr << "axis: " << e.get_axis_lengths() << std::endl;
		std::cerr << "--------" << std::endl;
	}
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
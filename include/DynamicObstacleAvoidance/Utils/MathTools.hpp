#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_MATHTOOLS_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_MATHTOOLS_H_

#include <vector>
#include <cstdlib>
#include <algorithm>
#include <eigen3/Eigen/Core>

namespace DynamicObstacleAvoidance
{
	namespace MathTools 
	{
		std::vector<double> linspace(const double& start, const double& end, const int& num);

		double rand_float(const double& a, const double& b=0);

		template <typename T> 
		int sign(T val) 
		{
	    	return (T(0) < val) - (val < T(0));
		}

		Eigen::Vector3d cartesian_to_polar(const Eigen::Vector3d& cartesian_point);

		Eigen::Vector3d polar_to_cartesian(const Eigen::Vector3d& cartesian_point);

		bool compare_theta(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs);

    	Eigen::MatrixXd sorted_cols_by_theta(Eigen::MatrixXd A);

    	std::pair<Eigen::Vector3d, Eigen::Vector3d> find_closest_points(Eigen::MatrixXd A, Eigen::Vector3d p, unsigned int index);
	}
}
#endif

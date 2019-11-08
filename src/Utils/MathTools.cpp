#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"

namespace DynamicObstacleAvoidance
{
    std::vector<double> MathTools::linspace(const double& start, const double& end, const int& num) {
        // catch rarely, throw often
        if (num < 2) {
            throw new std::exception();
        }
        int partitions = num - 1;
        std::vector<double> pts;
        // length of each segment    
        double length = (end - start) / partitions; 
        // first, not to change
        pts.push_back(start);
        for (int i = 1; i < num - 1; i ++) {
            pts.push_back(start + i * length);
        }
        // last, not to change
        pts.push_back(end);
        return pts;
    }

    double MathTools::rand_float(const double& a, const double& b) 
    {
        return ((double)rand() / RAND_MAX) * (a - b) + b;
    }

    Eigen::Vector3d MathTools::cartesian_to_polar(const Eigen::Vector3d& cartesian_point)
    {
        double radius = sqrt(cartesian_point(0) * cartesian_point(0) + cartesian_point(1) * cartesian_point(1) + cartesian_point(2) * cartesian_point(2));
        double theta = atan2(cartesian_point(1), cartesian_point(0));
        double phi = acos(cartesian_point(2) / radius);
        return Eigen::Vector3d(radius, theta, phi);
    }

    Eigen::Vector3d MathTools::polar_to_cartesian(const Eigen::Vector3d& cartesian_point)
    {
        double X = cos(cartesian_point(1)) * cos(cartesian_point(2)) * cartesian_point(0);
        double Y = sin(cartesian_point(1)) * cos(cartesian_point(2)) * cartesian_point(0);
        double Z = sin(cartesian_point(2)) * cartesian_point(0);
        return Eigen::Vector3d(X, Y, Z);
    }

    bool MathTools::compare_theta(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs)
    {
        return lhs(1) < rhs(1);
    }

    Eigen::MatrixXd MathTools::sorted_cols_by_theta(Eigen::MatrixXd A)
    {
        std::vector<Eigen::VectorXd> vec;
        for (int64_t i = 0; i < A.cols(); ++i)
            vec.push_back(A.col(i));

        std::sort(vec.begin(), vec.end(), &compare_theta);

        for (int64_t i = 0; i < A.cols(); ++i)
            A.col(i) = vec[i];

        return A;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> MathTools::find_closest_points(Eigen::MatrixXd A, Eigen::Vector3d p, unsigned int index)
    {
        std::vector<double> vec;
        for (int64_t i = 0; i < A.cols(); ++i) vec.push_back(A.col(i)(index));
        std::vector<double>::iterator it = std::lower_bound(std::begin(vec), std::end(vec), p(index));
        return std::make_pair(A.col(it - vec.begin()), A.col((it + 1) - vec.begin()));
    }
}

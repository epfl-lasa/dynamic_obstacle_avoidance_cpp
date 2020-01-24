#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"

namespace DynamicObstacleAvoidance
{
    std::vector<double> MathTools::linspace(double start, double end, int num) {
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

    std::vector<Eigen::Vector2d> MathTools::gridspace(std::pair<double, double> start, std::pair<double, double> end, std::pair<int, int> grid_size)
    {
        std::vector<double> linspace_x = MathTools::linspace(start.first, end.first, grid_size.first);
        std::vector<double> linspace_y = MathTools::linspace(start.second, end.second, grid_size.second);

        std::vector<Eigen::Vector2d> grid;
        grid.resize(grid_size.first * grid_size.second);
        unsigned int i = 0;
        for(auto& x:linspace_x)
        {
            for(auto& y:linspace_y)
            {
                Eigen::Vector2d point;
                point << x, y;
                grid[i] = point;
                ++i;
            }
        }
        return grid;
    }

    double MathTools::rand_float(double a, double b) 
    {
        return ((double)rand() / RAND_MAX) * (a - b) + b;
    }

    Eigen::Vector3d MathTools::cartesian_to_polar(const Eigen::Vector3d& cartesian_point)
    {
        double radius = sqrt(cartesian_point(0) * cartesian_point(0) + cartesian_point(1) * cartesian_point(1) + cartesian_point(2) * cartesian_point(2));
        double theta = acos(cartesian_point(2) / radius);
        double phi = atan2(cartesian_point(1), cartesian_point(0));
        return Eigen::Vector3d(radius, theta, phi);
    }

    Eigen::Vector3d MathTools::polar_to_cartesian(const Eigen::Vector3d& cartesian_point)
    {
        double X = cartesian_point(0) * sin(cartesian_point(1)) * cos(cartesian_point(2));
        double Y = cartesian_point(0) * sin(cartesian_point(1)) * sin(cartesian_point(2));
        double Z = cartesian_point(0) * cos(cartesian_point(1));
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

    std::pair<unsigned int, unsigned int> MathTools::find_closest_points(Eigen::MatrixXd A, Eigen::Vector3d p, unsigned int index)
    {
        std::vector<double> vec;
        for (int64_t i = 0; i < A.cols(); ++i) vec.push_back(A.col(i)(index));
        std::vector<double>::iterator it = std::lower_bound(std::begin(vec), std::end(vec), p(index));
        unsigned int first_index = it - vec.begin();
        unsigned int next_index = ((it + 1) - vec.begin()) % vec.size();
        return std::make_pair(first_index, next_index);
    }
}

#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"

std::vector<double> PlottingTools::linespace(const double& start, const double& ed, const int& num) {
    // catch rarely, throw often
    if (num < 2) {
        throw new std::exception();
    }
    int partitions = num - 1;
    std::vector<double> pts;
    // length of each segment    
    double length = (ed - start) / partitions; 
    // first, not to change
    pts.push_back(start);
    for (int i = 1; i < num - 1; i ++) {
        pts.push_back(start + i * length);
    }
    // last, not to change
    pts.push_back(ed);
    return pts;
}

void PlottingTools::plot_ellipsoids2D(const std::deque<Ellipsoid>& ellipsoids, bool is_show)
{
	for(Ellipsoid e:ellipsoids)
	{
		int n = 100;
		// use a linespace to have a full rotation angle between [-pi, pi]
		std::vector<double> alpha = linespace(0, 2*M_PI, n);
		// convert quaternion to AngleAxis
		Eigen::AngleAxisd orientation(e.get_orientation());
		Eigen::Vector3d axis = orientation.axis();
		double theta = (axis(2) > 0) ? orientation.angle() :  2*M_PI - orientation.angle();

		// use the parametric equation of an ellipse to draw
		std::vector<double> x(n);
		std::vector<double> y(n); 
		for (int i=0; i<n; ++i)
		{
			double a = alpha.at(i);
			x.at(i) = e.get_axis_lengths(0) * cos(a) * cos(theta) - e.get_axis_lengths(1) * sin(a) * sin(theta) + e.get_position()(0);
			y.at(i) = e.get_axis_lengths(0) * cos(a) * sin(theta) + e.get_axis_lengths(1) * sin(a) * cos(theta) + e.get_position()(1);
		}
		plt::plot(x, y);
		plt::plot({e.get_position()(0)}, {e.get_position()(1)}, "ko");
		plt::plot({e.get_reference_position()(0)}, {e.get_reference_position()(1)}, "kx");
	}

	if(is_show)
	{
		plt::xlim(-20, 20);
		plt::ylim(-20, 20);
		plt::show();
	}
}

void PlottingTools::plot_clusters(const std::deque<Eigen::MatrixXd>& clusters, bool is_show)
{
	for(Eigen::MatrixXd points:clusters)
	{
		std::vector<double> x(points.cols());
		std::vector<double> y(points.cols()); 
		for (int i=0; i<points.cols(); ++i)
		{
			x.at(i) = points.col(i)(0);
			y.at(i) = points.col(i)(1);
		}
		plt::plot(x, y, "o");
	}
	if(is_show)
	{
		plt::xlim(-20, 20);
		plt::ylim(-20, 20);
		plt::show();
	}
}

void PlottingTools::plot_fitted_clusters(const std::deque<Eigen::MatrixXd>& clusters, const std::deque<Ellipsoid>& ellipsoids, bool is_show)
{
	PlottingTools::plot_clusters(clusters, false);
	PlottingTools::plot_ellipsoids2D(ellipsoids, false);
	if(is_show)
	{
		plt::xlim(-20, 20);
		plt::ylim(-20, 20);
		plt::show();
	}
}

void PlottingTools::plot_configuration(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history)
{
	std::deque<Ellipsoid> ellipse_list;
	for(auto &o:obstacles)
	{
		Ellipsoid e = *dynamic_cast<Ellipsoid*>(o.get());
		ellipse_list.push_back(e);
	}
	PlottingTools::plot_ellipsoids2D(ellipse_list, false);
	plt::plot({agent.get_position()(0)}, {agent.get_position()(1)}, "bo");
	plt::plot({goal(0)}, {goal(1)}, "rx");

	std::vector<double> x(position_history.size());
	std::vector<double> y(position_history.size());
	int i=0;
	for(auto& point:position_history)
	{
		x.at(i) = point(0);
		y.at(i) = point(1);
		++i;
	}
	plt::plot(x, y, "k-");

	plt::xlim(-5, 5);
	plt::ylim(-5, 5);
	plt::show();
}
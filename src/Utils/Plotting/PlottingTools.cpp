#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"

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

void PlottingTools::plot_fitted_clusters(const std::deque<Eigen::MatrixXd>& clusters, const std::deque<std::unique_ptr<Obstacle> >& obstacles, bool is_show)
{
	PlottingTools::plot_clusters(clusters, false);
	for(auto& o:obstacles)
	{
		o->draw();
	}
	if(is_show)
	{
		plt::xlim(-20, 20);
		plt::ylim(-20, 20);
		plt::show();
	}
}

void PlottingTools::plot_configuration(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& savefile, const bool& is_show)
{
	plt::figure();
	for(auto &o:obstacles)
	{
		o->draw();
	}
	plt::plot({agent.get_position()(0)}, {agent.get_position()(1)}, "bo");
	agent.get_envelope().draw();
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
	plt::xlim(-1, 8);
	plt::ylim(-2, 8);
	if(savefile.compare(""))
	{
		std::string path = "/tmp/" + savefile + ".png";
		plt::save(path);
	}
	if(is_show)
	{
		plt::show();
	}
}
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"

namespace DynamicObstacleAvoidance
{
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
		std::vector<std::string> colors = {"b", "g", "r", "c", "m", "y"};
		plt::figure();
		unsigned int i = 0;
		for(auto &o:obstacles)
		{
			o->draw(colors[i]);
			i = (i + 1) % colors.size();
		}
		plt::plot({agent.get_position()(0)}, {agent.get_position()(1)}, "bo");
		plt::plot({goal(0)}, {goal(1)}, "rx");

		std::vector<double> x(position_history.size());
		std::vector<double> y(position_history.size());
		i = 0;
		for(auto& point:position_history)
		{
			x.at(i) = point(0);
			y.at(i) = point(1);
			++i;
		}
		plt::plot(x, y, "k-");
		plt::xlim(-10, 10);
		plt::ylim(-10, 10);
		if(savefile.compare(""))
		{
			std::string path = "./tmp/" + savefile + ".png";
			plt::save(path);
		}
		if(is_show)
		{
			plt::show();
		}
		plt::close();
	}

	void PlottingTools::plot_configuration(const std::deque<std::unique_ptr<Obstacle> >& obstacles, const std::string& savefile, const bool& is_show)
	{
		std::vector<std::string> colors = {"b", "g", "r", "c", "m", "y"};
		plt::figure();
		unsigned int i = 0;
		for(auto &o:obstacles)
		{
			o->draw(colors[i]);
			i = (i + 1) % colors.size();
		}
		plt::xlim(-10, 10);
		plt::ylim(-10, 10);
		if(savefile.compare(""))
		{
			std::string path = "./tmp/" + savefile + ".png";
			plt::save(path);
		}
		if(is_show)
		{
			plt::show();
		}
		plt::close();
	}

	void PlottingTools::plot_configuration(const std::deque<std::unique_ptr<Obstacle> >& obstacles, const std::vector<Eigen::Array4d> quivers, const Eigen::Vector3d& goal, const std::string& savefile, const bool& is_show)
	{
		std::vector<std::string> colors = {"b", "g", "r", "c", "m", "y"};
		plt::figure();
		unsigned int i = 0;
		for(auto &o:obstacles)
		{
			o->draw(colors[i]);
			i = (i + 1) % colors.size();
		}
		plt::xlim(-10, 10);
		plt::ylim(-10, 10);

		// plot quivers
		std::vector<double> x, y, u, v;
		for(auto& vec:quivers)
		{
			x.push_back(vec(0));
			y.push_back(vec(1));
			u.push_back(vec(2));
			v.push_back(vec(3));
		}
		plt::quiver(x, y, u, v);

		// plot goal
		plt::plot({goal(0)}, {goal(1)}, "rx");

		// save the plot
		if(savefile.compare(""))
		{
			std::string path = "./tmp/" + savefile + ".png";
			plt::save(path);
		}
		if(is_show)
		{
			plt::show();
		}
		plt::close();
	}
}
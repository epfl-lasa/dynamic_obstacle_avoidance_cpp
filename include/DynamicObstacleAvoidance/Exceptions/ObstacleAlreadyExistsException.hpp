#ifndef DYNAMICOBSTACLEAVOIDANCE_EXCEPTIONS_OBSTACLEALREADYEXISTSEXCEPTION_H_
#define DYNAMICOBSTACLEAVOIDANCE_EXCEPTIONS_OBSTACLEALREADYEXISTSEXCEPTION_H_

#include <iostream>
#include <exception>

namespace DynamicObstacleAvoidance
{
	namespace Exceptions
	{
		class ObstacleAlreadyExistsException: public std::runtime_error
		{
		public:
			explicit ObstacleAlreadyExistsException(const std::string& obstacle_name) :
			runtime_error("Obstacle " + obstacle_name + " already exists in the environment")
			{};
		};
	}
}
#endif
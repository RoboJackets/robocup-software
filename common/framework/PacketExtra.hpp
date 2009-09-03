#pragma once

#include "Obstacle.hpp"
#include "Path.hpp"
#include "ConfigFile.hpp"

namespace Framework
{	
	class Robot
	{
		public:	
			ObstacleGroup obstacles;
			
			ConfigFile::Robot config;
	};
}

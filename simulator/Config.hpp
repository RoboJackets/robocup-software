#pragma once

#include <QString>
#include <QDomElement>

#include "Physics/Env.hpp"

/** Simulation config file loader */
class Config
{	
	public:
		Config(QString filename, Env* env);
		
	private:
		void procTeam(QDomElement e, bool blue);
		
		Env* _env;
};

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <QString>
#include <QDomElement>

#include "Physics/Env.hpp"

/** Simulation config file loader */
class Config
{	
	public:
		Config(QString filename, Env* env);
		
	private:
		void procTeam(QDomElement e, Team t);
		
		Env* _env;
};

#endif /* CONFIG_HPP_ */

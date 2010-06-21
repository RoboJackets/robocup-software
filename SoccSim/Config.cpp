#include "Config.hpp"

#include <QDomDocument>
#include <QDomAttr>
#include <QDebug>
#include <QFile>

#include <stdexcept>

Config::Config(QString filename, Env* env) :
	_env(env)
{
	//load the config file
	QFile configFile(filename);
	
	QDomDocument _doc;
	
	if (!configFile.open(QIODevice::ReadOnly))
	{
		throw std::runtime_error("Unable to open config file.");
	}

	if (!_doc.setContent(&configFile))
	{
		configFile.close();
		throw std::runtime_error("Internal: unable to set document content.");
	}
	configFile.close();
	
	//load rest of file
	qDebug() << "Loading config: " << filename;
	
	QDomElement root = _doc.documentElement();
	
	if (root.isNull() || root.tagName() != QString("simulation"))
	{
		throw std::runtime_error("Document format: expected <motion> tag");
	}
	
	QDomElement element = root.firstChildElement();
		
	while (!element.isNull())
	{
		if (element.tagName() == QString("ball"))
		{
			float x = element.attribute("x").toFloat();
			float y = element.attribute("y").toFloat();
			
			_env->addBall(Geometry2d::Point(x,y));
		}
		else if (element.tagName() == QString("blue"))
		{
			procTeam(element, Blue);
		}
		else if (element.tagName() == QString("yellow"))
		{
			procTeam(element, Yellow);
		}
		
		element = element.nextSiblingElement();
	}
}

void Config::procTeam(QDomElement e, Team t)
{
	QDomElement elem = e.firstChildElement();
	
	while (!elem.isNull())
	{
		if (elem.tagName() == "robot")
		{
			float x = elem.attribute("x").toFloat();
			float y = elem.attribute("y").toFloat();
			int id = elem.attribute("id").toInt();
			
			if (elem.hasAttribute("rev")) {
				QString rev = elem.attribute("rev");
				Robot::Rev r;
				if (rev.contains("2008"))
				{
				    r = Robot::rev2008;
				} else if (rev.contains("2010")) {
				    r = Robot::rev2010;
				}
				_env->addRobot(t, id, Geometry2d::Point(x, y), r);
			} else {
				_env->addRobot(t, id, Geometry2d::Point(x, y), Robot::rev2008);
			}
		}
		
		elem = elem.nextSiblingElement();
	}
}

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
			
			_env->addBall(x, y);
		}
		else if (element.tagName() == QString("blue") || 
				element.tagName() == QString("yellow"))
		{
			procTeam(element);
		}
		else if (element.tagName() == QString("vision"))
		{
			
		}
		else if (element.tagName() == QString("radio"))
		{
			
		}
		
		element = element.nextSiblingElement();
	}
}

void Config::procTeam(QDomElement e)
{
	QDomElement elem = e.firstChildElement();
	
	while (!elem.isNull())
	{
		if (elem.tagName() == "robot")
		{
			float x = elem.attribute("x").toFloat();
			float y = elem.attribute("y").toFloat();
			int id = elem.attribute("id").toInt();
			
			_env->addRobot(id, x, y);
		}
		else if (elem.tagName() == "robots")
		{
			
		}
		
		elem = elem.nextSiblingElement();
	}
}

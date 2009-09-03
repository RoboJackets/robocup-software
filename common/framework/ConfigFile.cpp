#include "ConfigFile.hpp"

#include <QDomDocument>
#include <QDomElement>
#include <QDomAttr>
#include <QFile>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

ConfigFile::ConfigFile(QString filename) :
	_filename(filename), _doc("config")
{
}

ConfigFile::~ConfigFile()
{
}

void ConfigFile::load() throw (std::runtime_error)
{
	QFile configFile(_filename);
	
	if (!configFile.open(QIODevice::ReadOnly))
	{
		throw std::runtime_error("Unable to open config file.");
	}

	QString errorMsg;
	int errorLine = 0, errorColumn = 0;
	if (!_doc.setContent(&configFile, &errorMsg, &errorLine, &errorColumn))
	{
		configFile.close();
		throw runtime_error(str(format("line %d col %d: %s") % errorLine
		        % errorColumn % errorMsg.toStdString()));
	}
	configFile.close();
	
	
	//load rest of file
	qDebug() << "Loading config: " << _filename;
	
	QDomElement root = _doc.documentElement();
	
	if (root.isNull() || root.tagName() != QString("config"))
	{
		throw std::runtime_error("Document format: expected <config> tag");
	}

	QDomElement element = root.firstChildElement();
	
	while (!element.isNull())
	{
		if (element.tagName() == "robots")
		{
			procRobots(element);
		}
		else if (element.tagName() == "worldModel")
		{
			//procWorldModel(element);
			worldModel.proc(element);
		}
		else if (element.tagName() == "motionModule")
		{
			motionModule.proc(element);
		}

		element = element.nextSiblingElement();
	}
}

void ConfigFile::save(QString filename) throw (std::runtime_error)
{
    QFile configFile(_filename);
    if(!(filename == QString("")))
    {
        QFile configFile(filename);
    }

    if (!configFile.open(QIODevice::WriteOnly))
    {
        throw std::runtime_error("Unable to open config file.");
    }
    else
    {
        configFile.write(_doc.toByteArray());
    }
    configFile.close();

    qDebug() << "Wrote: " << filename;
}

/// World Model

void ConfigFile::WorldModel::proc(QDomElement element)
{
    QDomElement posE = element.firstChildElement("pos");
    QDomElement angleE = element.firstChildElement("angle");
    
    if (!posE.isNull())
    {
    	pos.proc(posE.firstChild().toElement());
    }
    
    if (!angleE.isNull())
	{
    	angle.proc(angleE.firstChild().toElement());
	}
}

void ConfigFile::WorldModel::Filter::proc(QDomElement element)
{
	if (element.tagName() == "filter")
	{	
		alpha = valueFloat(element.attributeNode("alpha"));
		beta = valueFloat(element.attributeNode("beta"));
		gamma = valueFloat(element.attributeNode("gamma"));
	}
}

void ConfigFile::procRobots(QDomElement element)
{
	QDomElement robot = element.firstChildElement();
	
	while (!robot.isNull())
	{
		if (robot.tagName() == "robot")
		{
			//new robot
			unsigned int id = ConfigFile::valueUInt(robot.attributeNode("id"));
			
			ConfigFile::Robot* r = new ConfigFile::Robot();
			_robots[id] = r;
			
			r->proc(robot);
		}

		robot = robot.nextSiblingElement();
	}
}

/// Robot
void ConfigFile::Robot::proc(QDomElement element)
{
	QDomElement motionE = element.firstChildElement("motion");
	QDomElement kickerE = element.firstChildElement("kicker");
	
	motion.proc(motionE);
	kicker.proc(kickerE);
}

/// Motion

void ConfigFile::Robot::Motion::proc(QDomElement element)
{
	QDomElement child = element.firstChildElement();
	
	while (!child.isNull())
	{
		const QString& name = child.tagName();
		if (name == "translation")
		{
			QDomElement deg0E = child.firstChildElement("deg");
			QDomElement deg1E = deg0E.nextSiblingElement("deg");
			
			if (!deg0E.isNull() && !deg1E.isNull())
			{
				if (deg0E.attribute("value") == "0")
				{
					deg0.proc(deg0E.firstChild().toElement());
					deg45.proc(deg1E.firstChild().toElement());
				}
				else if (deg0E.attribute("value") == "45")
				{
					deg0.proc(deg1E.firstChild().toElement());
					deg45.proc(deg0E.firstChild().toElement());
				}
			}
			
			pos.proc(child.firstChildElement("pid"));
		}
		else if (name == "rotation")
		{
			rotation.proc(child.firstChildElement("dynamics"));
			angle.proc(child.firstChildElement("pid"));
		}
		
		child = child.nextSiblingElement();
	}
}

void ConfigFile::MotionModule::proc(QDomElement element)
{
	robot.proc(element.firstChildElement("robot"));
}

void ConfigFile::MotionModule::Robot::proc(QDomElement element)
{
	procAxels(element.firstChildElement("axles"));
}

void ConfigFile::MotionModule::Robot::procAxels(QDomElement element)
{
    QDomElement axle = element.firstChildElement();
	
    while (!axle.isNull())
    {
        if (axle.tagName() == QString("axle"))
        {
            //get x and y parameters
            float x = valueFloat(axle.attributeNode("x"));
            float y = valueFloat(axle.attributeNode("y"));

            Geometry2d::Point axle(x, y);
            //rotate by -90 to go from robot coord to team space
            axles.append(axle.normalized());
        }
        else
        {
            throw std::runtime_error("Only an <axle> tag can be used between <axles></axles>");
        }

        axle = axle.nextSiblingElement();
    }
}

void ConfigFile::Robot::Motion::Dynamics::proc(QDomElement element)
{
	if (element.tagName() == "dynamics")
	{
		velocity = valueFloat(element.attributeNode("velocity"));
		acceleration = valueFloat(element.attributeNode("acceleration"));
		deceleration = valueFloat(element.attributeNode("deceleration"));
	}
}

void ConfigFile::Robot::Motion::Pid::proc(QDomElement element)
{
	if (element.tagName() == "pid")
	{
		p = valueFloat(element.attributeNode("p"));
		i = valueFloat(element.attributeNode("i"));
		d = valueFloat(element.attributeNode("d"));
	}
}
void ConfigFile::Robot::Kicker::proc(QDomElement element)
{
	m = valueFloat(element.attributeNode("m"));
	b = valueFloat(element.attributeNode("b"));
}

#if 0
//TODO Replace with templated member function
void ConfigFile::setElement(QString tagString,int value)
{
    QStringList tagNamesList = tagString.split(",");

    //Initialize element to the document root
    QDomElement element = _doc.documentElement();

    QStringList::const_iterator i;
    for (i = tagNamesList.constBegin(); i != tagNamesList.constEnd(); ++i)
    {
        QDomNodeList elemList = element.elementsByTagName(*i);
        element = elemList.at(0).toElement();
    }

    element.setAttribute("value",value);

}

void ConfigFile::setElement(QString tagString,double value)
{
    QStringList tagNamesList = tagString.split(",");

    //Initialize element to the document root
    QDomElement element = _doc.documentElement();

    QStringList::const_iterator i;
    for (i = tagNamesList.constBegin(); i != tagNamesList.constEnd(); ++i)
    {
        QDomNodeList elemList = element.elementsByTagName(*i);
        element = elemList.at(0).toElement();
    }

    element.setAttribute("value",value);
}
#endif

/// value to type conversions ///

float ConfigFile::valueFloat(QDomAttr attr)
{
    QString v = attr.value();

    bool ok = false;
    float val = v.toFloat(&ok);

    if (ok)
    {
        return val;
    }
    else
    {
        throw std::runtime_error("Attribute value not a valid float.");
    }
}

uint ConfigFile::valueUInt(QDomAttr attr)
{
    QString v = attr.value();

    bool ok = false;
    uint val = v.toUInt(&ok);

    if (ok)
    {
        return val;
    }
    else
    {
        throw std::runtime_error("Attribute value not a valid int.");
    }
}

bool ConfigFile::valueBool(QDomAttr attr)
{
  QString v = attr.value();
  return v == "true";
}

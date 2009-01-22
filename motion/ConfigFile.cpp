#include "ConfigFile.hpp"

#include <QDomDocument>
#include <QDomElement>
#include <QDomAttr>
#include <QFile>
#include <QDebug>
#include <QString>
#include <QStringList>

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
        QString errorMsg = "Hello";
        int errorLine, errorColumn;

	if (!configFile.open(QIODevice::ReadOnly))
	{
		throw std::runtime_error("Unable to open config file.");
	}

	if (!_doc.setContent(&configFile,1,&errorMsg,&errorLine,&errorColumn))
	{
                printf(errorMsg.toAscii());
                printf("\nLine %d , Column %d \n", errorLine, errorColumn);
                configFile.close();
		throw std::runtime_error("Internal: unable to set document content.");
	}
	configFile.close();

	//load rest of file
	qDebug() << "Loading config: " << _filename;

	QDomElement root = _doc.documentElement();

	if (root.isNull() || root.tagName() != QString("motion"))
	{
		throw std::runtime_error("Document format: expected <motion> tag");
	}

	QDomElement element = root.firstChildElement();

	while (!element.isNull())
	{
		if (element.tagName() == QString("pid"))
		{
			procPID(element);
		}
		else if (element.tagName() == QString("axels"))
		{
			_axels.clear();
			procAxels(element);
		}

		element = element.nextSiblingElement();
	}
}

void ConfigFile::save()
{

}

/** process a pid tag */
void ConfigFile::procPID(QDomElement element)
{
	QDomAttr nameAttr = element.attributeNode("name");
	QString pidName = nameAttr.value();

	//qDebug() << "\tPID Element: " << pidName;

	PidInfo* pid = &_pos;

	if (pidName == "angle")
	{
		pid = &_angle;
	}

	QDomElement eElem = element.firstChildElement("p");
	if (!eElem.isNull())
	{
		pid->p = valueFloat(eElem.attributeNode("value"));
	}

	eElem = element.firstChildElement("i");
	if (!eElem.isNull())
	{
		pid->i = valueFloat(eElem.attributeNode("value"));
	}

	eElem = element.firstChildElement("d");
	if (!eElem.isNull())
	{
		pid->d = valueFloat(eElem.attributeNode("value"));
	}

	eElem = element.firstChildElement("windup");
	if (!eElem.isNull())
	{
		pid->windup = valueUInt(eElem.attributeNode("value"));
	}

	//qDebug("\t\tP: %f\tI: %f\tD: %f\t W: %i", pid->p, pid->i, pid->d, pid->windup);
}

void ConfigFile::procAxels(QDomElement element)
{
	QDomElement axel = element.firstChildElement();

	while (!axel.isNull())
	{
		if (axel.tagName() == QString("axel"))
		{
			//get x and y parameters
			float x = valueFloat(axel.attributeNode("x"));
			float y = valueFloat(axel.attributeNode("y"));

			Geometry::Point2d axel(x, y);

			//rotate by -90 to go from robot coord to team space
			axel.rotate(Geometry::Point2d(0,0), -90);
			_axels.append(axel.norm());
			//qDebug("Axel: %f %f", x, y);
		}
		else
		{
			throw std::runtime_error("Only an <axel> tag can be used between <axels></axels>");
		}

		axel = axel.nextSiblingElement();
	}
}

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
		//TODO indicate which attribute
		throw std::runtime_error("Attribute value not a valid float.");
	}
}

int ConfigFile::valueUInt(QDomAttr attr)
{
	QString v = attr.value();

	bool ok = false;
	int val = v.toUInt(&ok);

	if (ok)
	{
		return val;
	}
	else
	{
		//TODO indicate which attribute
		throw std::runtime_error("Attribute value not a valid int.");
	}
}

ConfigFile::RobotCfg ConfigFile::robotConfig(const unsigned int id)
{
	RobotCfg cfg;
	cfg.id = id;

	cfg.posPid = _pos;
	cfg.anglePid = _angle;
	cfg.axels = _axels;

	return cfg;
}

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
    qDebug() << "Loading config: " << _filename;

    QDomElement root = _doc.documentElement();

    if (root.isNull() || root.tagName() != QString("motion"))
    {
        throw std::runtime_error("Document format: expected <motion> tag");
    }

    QDomElement element = root.firstChildElement();

    while (!element.isNull())
    {
        if (element.tagName() == QString("linearController"))
        {
            procLinearController(element);
        }
        else if (element.tagName() == QString("robotData"))
        {
            procRobotData(element);
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

void ConfigFile::procLinearController(QDomElement element)
{
    QDomAttr nameAttr = element.attributeNode("name");
    QString name = nameAttr.value();

    linearControllerInfo* cntrlr = &_pos;

    if (name == "angle")
    {
        cntrlr = &_angle;
    }

    QDomElement eElem = element.firstChildElement("kp");
    if (!eElem.isNull())
    {
        cntrlr->Kp = valueFloat(eElem.attributeNode("value"));
    }

    eElem = element.firstChildElement("kd");
    if (!eElem.isNull())
    {
        cntrlr->Kd = valueFloat(eElem.attributeNode("value"));
    }

    eElem = element.firstChildElement("deadband");
    if (!eElem.isNull())
    {
        cntrlr->deadband.x = valueFloat(eElem.attributeNode("x"));
        cntrlr->deadband.y = valueFloat(eElem.attributeNode("y"));
    }
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
        }
        else
        {
            throw std::runtime_error("Only an <axel> tag can be used between <axels></axels>");
        }

        axel = axel.nextSiblingElement();
    }
}

void ConfigFile::procRobotData(QDomElement element)
{
    QDomElement childElement = element.firstChildElement();
    if (childElement.tagName() == QString("axels"))
    {
        _axels.clear();
        procAxels(childElement);
    }

    QDomElement eElem = element.firstChildElement("maxAccel");
    if (!eElem.isNull())
    {
        _maxAccel = valueFloat(eElem.attributeNode("value"));
    }

    eElem = element.firstChildElement("maxWheelVel");
    if (!eElem.isNull())
    {
        _maxWheelVel = valueFloat(eElem.attributeNode("value"));
    }

    eElem = element.firstChildElement("maxRobotVel");
    if (!eElem.isNull())
    {
        _maxRobotVel = valueFloat(eElem.attributeNode("value"));
    }

}

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

ConfigFile::RobotCfg ConfigFile::robotConfig()
{
    RobotCfg cfg;

    cfg.posCntrlr = _pos;
    cfg.angleCntrlr = _angle;
    cfg.axels = _axels;

    cfg.maxAccel = _maxAccel;
    cfg.maxWheelVel = _maxWheelVel;
    cfg.maxRobotVel = _maxRobotVel;

    return cfg;
}

ConfigFile::RobotFilterCfg ConfigFile::robotFilterConfig()
{
    RobotFilterCfg cfg;
    cfg.kf_enable = false;

    return cfg;
}



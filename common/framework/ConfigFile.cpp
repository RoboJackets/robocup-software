#include "ConfigFile.hpp"

#include <iostream>

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
		else if (element.tagName() == "revLUT")
		{
			procRevLUT(element);
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

ConfigFile::shared_robot ConfigFile::robot(unsigned int id) const
{
	bool verbose = false;
	if (verbose) cout << "Getting id for robot " << id << ": ";
	// look for overrides
	map<unsigned int, shared_robot>::const_iterator override_it = _robot_overrides.find(id);
	if (override_it != _robot_overrides.end()) {
		if (verbose) cout << "   override" << endl;
		return override_it->second;
	}


	// use defaults
	map<unsigned int, RobotRev>::const_iterator rev_it = _revisionLUT.find(id);
	if (rev_it == _revisionLUT.end()) {
		if (verbose) cout << "default - 2008" << endl;
		return _defaultRobot2008; // NOTE: this sets default behavior to assume rev2008
	} else if (rev_it->second == rev2008) {
		if (verbose) cout << "2008" << endl;
		return _defaultRobot2008;
	} else if (rev_it->second == rev2010) {
		if (verbose) cout << "2010" << endl;
		return _defaultRobot2010;
	} else {
		throw invalid_argument("Robot does not exist!");
	}
}

/// World Model

void ConfigFile::WorldModel::proc(QDomElement element)
{
    QDomElement posE = element.firstChildElement("pos");
    QDomElement angleE = element.firstChildElement("angle");
    QDomElement abgModelRobotE = element.firstChildElement("abgModelRobot");
    QDomElement kalmanModelRobotE = element.firstChildElement("kalmanModelRobot");
    QDomElement rbpfModelBallRollingE = element.firstChildElement("rbpfModelBallRolling");
    QDomElement rbpfModelBallKickedE = element.firstChildElement("rbpfModelBallKicked");
    
    if (!posE.isNull())
    {
    	pos.proc(posE.firstChild().toElement());
    }
    
    if (!angleE.isNull())
	{
    	angle.proc(angleE.firstChild().toElement());
	}

    if (!abgModelRobotE.isNull())
    {
    	abgModelRobot.proc(abgModelRobotE.firstChild().toElement());
    }

    if (!kalmanModelRobotE.isNull())
    {
    	kalmanModelRobot.proc(kalmanModelRobotE.firstChild().toElement());
    }

    if (!rbpfModelBallRollingE.isNull())
    {
    	rbpfModelBallRolling.proc(rbpfModelBallRollingE.firstChild().toElement());
    }

    if (!rbpfModelBallKickedE.isNull())
    {
    	rbpfModelBallKicked.proc(rbpfModelBallKickedE.firstChild().toElement());
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

void ConfigFile::WorldModel::ABGModelRobot::proc(QDomElement element)
{
	if (element.tagName() == "abgModelRobot")
	{
		alphaPos = valueFloat(element.attributeNode("alphaPos"));
		betaPos = valueFloat(element.attributeNode("betaPos"));
		gammaPos = valueFloat(element.attributeNode("gammaPos"));
		alphaAng = valueFloat(element.attributeNode("alphaAng"));
		betaAng = valueFloat(element.attributeNode("betaAng"));
		gammaAng = valueFloat(element.attributeNode("gammaAng"));
	}
}

void ConfigFile::WorldModel::KalmanModelRobot::proc(QDomElement element)
{
	if (element.tagName() == "kalmanModelRobot")
	{
		covPosVel = valueFloat(element.attributeNode("covPosVel"));
		covVelAcc = valueFloat(element.attributeNode("covVelAcc"));
		covPosAcc = valueFloat(element.attributeNode("covPosAcc"));
		covPos = valueFloat(element.attributeNode("covPos"));
		covVel = valueFloat(element.attributeNode("covVel"));
		covAcc = valueFloat(element.attributeNode("covAcc"));
		measurementNoise = valueFloat(element.attributeNode("measurementNoise"));
	}
}

void ConfigFile::WorldModel::RBPFModelBallRolling::proc(QDomElement element)
{
	if (element.tagName() == "rbpfModelBallRolling")
	{
		processNoiseSqrdPos = valueFloat(element.attributeNode("processNoiseSqrdPos"));
		processNoiseSqrdVel = valueFloat(element.attributeNode("processNoiseSqrdVel"));
		processNoiseSqrdAcc = valueFloat(element.attributeNode("processNoiseSqrdAcc"));
		measurementNoiseSqrd = valueFloat(element.attributeNode("measurementNoiseSqrd"));
	}
}

void ConfigFile::WorldModel::RBPFModelBallKicked::proc(QDomElement element)
{
	if (element.tagName() == "rbpfModelBallKicked")
	{
		processNoiseSqrdPos = valueFloat(element.attributeNode("processNoiseSqrdPos"));
		processNoiseSqrdVel = valueFloat(element.attributeNode("processNoiseSqrdVel"));
		processNoiseSqrdAcc = valueFloat(element.attributeNode("processNoiseSqrdAcc"));
		measurementNoiseSqrd = valueFloat(element.attributeNode("measurementNoiseSqrd"));
	}
}

void ConfigFile::procRobots(QDomElement element)
{
	bool verbose = true;
	QDomElement robot = element.firstChildElement();
	
	while (!robot.isNull())
	{
		if (robot.tagName() == "robot")
		{
			//new robot
			unsigned int id = ConfigFile::valueUInt(robot.attributeNode("id"));
			if (verbose) cout << "ConfigFile: new robot - " << id << endl;
			QString revString = robot.attributeNode("rev").value();
			RobotRev rev = rev2008; // use default value of 2008 robots
			if (revString == "rev2010") {
				if (verbose) cout << "    found 2010 robot" << endl;
				rev = rev2010;
			} else if (revString == "rev2008") {
				if (verbose) cout << "    found 2008 robot" << endl;
				rev = rev2008;
			}
			
			shared_robot r(new ConfigFile::Robot());
			r->rev = rev;
			r->proc(robot);

			// detect defaults
			if (id == 2008) {
				_defaultRobot2008 = r;
			} else if (id == 2010) {
				_defaultRobot2010 = r;
			} else {
				if (verbose) cout << "    adding to override list" << endl;
				// treat as override otherwise
				_robot_overrides[id] = r; // add to override list
				_revisionLUT[id] = rev;   // remember revision
			}
		}
		robot = robot.nextSiblingElement();
	}
}

/// Lookup for revisions
void ConfigFile::procRevLUT(QDomElement element) {
	bool verbose = true;
	if (verbose) cout << "Processing LUT" << endl;
	QDomElement child = element.firstChildElement();

	while (!child.isNull())
	{
		const QString& name = child.tagName();
		if (name == "rev2010")
		{
			if (verbose) cout <<  "Found 2010 robots: ";
			// loop over list
			QDomElement list_mem = child.firstChildElement();
			while (!list_mem.isNull()) {
				uint id = ConfigFile::valueUInt(list_mem.attributeNode("id"));
				_revisionLUT[id] = rev2010;
				if (verbose) cout << id << " ";
				list_mem = list_mem.nextSiblingElement();
			}
			cout << endl;
		}
		else if (name == "rev2008")
		{
			if (verbose) cout <<  "Found 2008 robots: ";
			// loop over list
			QDomElement list_mem = child.firstChildElement();
			while (!list_mem.isNull()) {
				uint id = ConfigFile::valueUInt(list_mem.attributeNode("id"));
				_revisionLUT[id] = rev2008;
				if (verbose) cout << id << " ";
				list_mem = list_mem.nextSiblingElement();
			}
			cout << endl;
		}

		child = child.nextSiblingElement();
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
		}
		else if (name == "rotation")
		{
			rotation.proc(child.firstChildElement("dynamics"));
			angle.proc(child.firstChildElement("pid"));
		}
		else if (name == "control")
		{
			wheel.proc(child.firstChildElement("pid"));
		}
		else if (name == "coeffs")
		{
			// initialize the coefficients
			QDomElement list_mem = child.firstChildElement();
			while (!list_mem.isNull()) {
				float val = ConfigFile::valueFloat(list_mem.attributeNode("value"));
				output_coeffs.push_back(val);
				list_mem = list_mem.nextSiblingElement();
			}
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

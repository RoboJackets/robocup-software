#ifndef CONFIGFILE_HPP_
#define CONFIGFILE_HPP_

#include <QString>
#include <QDomDocument>
#include <QVector>

#include <stdexcept>

#include <Geometry/Point2d.hpp>

#include "Pid.hpp"

class ConfigFile
{
	//types
	public:
		typedef struct
		{
			float p,i,d;
			unsigned int windup;
		} PidInfo;
		
		typedef struct
		{
			unsigned int id;
			
			PidInfo posPid, anglePid;
			
			QVector<Geometry::Point2d> axels;
		} RobotCfg;
	
	public:
		ConfigFile(QString filename);
		~ConfigFile();
		
		void load() throw(std::runtime_error);
		void save();
		
		RobotCfg robotConfig(const unsigned int id);
		
		const QVector<Geometry::Point2d> axels() const { return _axels; }
		
	protected:
		/** returns the value of the attribute */
		static float valueFloat(QDomAttr attr);
		static int valueUInt(QDomAttr attr);
		
		/** process a pid tag */
		void procPID(QDomElement element);
		/** process the axels */
		void procAxels(QDomElement element);
		
	private:
		QString _filename;
		QDomDocument _doc;
		
		/** pid info for position and angle */
		PidInfo _pos, _angle;
		
		/** axel positions */
		QVector<Geometry::Point2d> _axels;
};

#endif /* CONFIGFILE_HPP_ */

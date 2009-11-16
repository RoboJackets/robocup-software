#pragma once

#include <QString>
#include <QDomDocument>
#include <QVector>
#include <QStringList>

#include <stdexcept>
#include <map>

#include <../Utils.hpp>
#include <Geometry2d/Point.hpp>

class ConfigFile
{
	public:				
		/** robot configuration */
		class Robot
		{
			public:
				class Motion
				{
					public:
						class Pid
						{
							public:
								Pid()
								{
									p = i = d = 0;
								}
								
								float p;
								float i;
								float d;
								
								void proc(QDomElement element);
						};
						
						class Dynamics
						{
							public:
								Dynamics()
								{
									velocity = 0;
									acceleration = 0;
									deceleration = 0;
								}
								
								//velocity forward
								float velocity;
								
								//acceleration forward
								float acceleration;
								
								float deceleration;
								
								void proc(QDomElement element);
						};
						
						//dynamics information
						//forward and 45 degree direction
						Dynamics deg0;
						Dynamics deg45;
						Dynamics rotation;
						
						Pid pos;
						Pid angle;
						
						void proc(QDomElement element);
				};
				
				class Kicker
				{
					public:
						Kicker()
						{
							m = b = 0;
						}
						
						float m;
						float b;
						
						void proc(QDomElement element);
				};
				
				Motion motion;
				Kicker kicker;
				
				void proc(QDomElement element);
		};
		
		class MotionModule
		{
			public:
				class Robot
				{
					public:
						QVector<Geometry2d::Point> axles;
						
						void proc(QDomElement element);
						void procAxels(QDomElement element);
				};
				
				void proc(QDomElement element);
				
				Robot robot;
		};
		
		class WorldModel
		{
			public:
				class Filter
				{
					public:
						Filter()
						{
							alpha = beta = gamma = 1;
						}
						
						float alpha;
						float beta;
						float gamma;
						
						void proc(QDomElement element);
				};
				
				//position filter
				Filter pos;
				
				//angle filter
				Filter angle;
				
				void proc(QDomElement element);
		};

	public:
		ConfigFile(QString filename);
		~ConfigFile();

		void load() throw(std::runtime_error);
		void save(QString filename = QString("")) throw (std::runtime_error);
		
		//void setElement(QString tagString, int value);
		//void setElement(QString tagString, double value);
		
		ConfigFile::Robot* robot(unsigned int id) const
		{
			return Utils::map_lookup(_robots, id);
		}
		
		WorldModel worldModel;
		MotionModule motionModule;
		
	protected:
		/** returns the value of the attribute */
		static float valueFloat(QDomAttr attr);
		static uint valueUInt(QDomAttr attr);
		static bool valueBool(QDomAttr attr);

		void procRobots(QDomElement element);
				
	private:
		QString _filename;
		QDomDocument _doc;
		
		//robot configurations
		std::map<unsigned int, ConfigFile::Robot*> _robots;
};

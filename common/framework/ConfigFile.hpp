#pragma once

#include <boost/shared_ptr.hpp>

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
		/** label revision of robot */
		typedef enum {
			rev2008 = 0,
			rev2010 = 1
		} RobotRev;

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
						}; // \class Pid
						
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
						}; // \class Dynamics
						
						//dynamics information
						//forward and 45 degree direction
						Dynamics deg0;
						Dynamics deg45;
						Dynamics rotation;
						
						// only using PID for angular control
						Pid angle;
						
						// need coefficients for the output FIR filter
						std::vector<float> output_coeffs;

						void proc(QDomElement element);
				}; // \class motion
				
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
				
				RobotRev rev;

				Motion motion;
				Kicker kicker;
				
				void proc(QDomElement element);
		}; // \class Robot
		
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
		typedef boost::shared_ptr<ConfigFile::Robot> shared_robot;

		ConfigFile(QString filename);
		~ConfigFile();

		void load() throw(std::runtime_error);
		void save(QString filename = QString("")) throw (std::runtime_error);
		
		QString filename() const
		{
			return _filename;
		}

		//void setElement(QString tagString, int value);
		//void setElement(QString tagString, double value);
		
		/** gets the value for a particular robot and handles defaults */
		shared_robot robot(unsigned int id) const;
		
		WorldModel worldModel;
		MotionModule motionModule;
		
		/** Access for gui interface */
		shared_robot defaultRobot2008() const { return _defaultRobot2008; }
		shared_robot defaultRobot2010() const { return _defaultRobot2010; }

	protected:
		/** returns the value of the attribute */
		static float valueFloat(QDomAttr attr);
		static uint valueUInt(QDomAttr attr);
		static bool valueBool(QDomAttr attr);

		void procRobots(QDomElement element);
		void procRevLUT(QDomElement element);
				
	private:
		QString _filename;
		QDomDocument _doc;
		
		/**
		 * robot configurations
		 *  - we keep default configurations
		 *  and the config file will include a lookup for robot type
		 */
		shared_robot _defaultRobot2008, _defaultRobot2010;
		std::map<unsigned int, RobotRev> _revisionLUT;
		std::map<unsigned int, shared_robot> _robot_overrides;
}; // \class ConfigFile

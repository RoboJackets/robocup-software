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
						
						// wheel PIDs for encoder use
						Pid wheel;

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
				
				Robot()
				{
				    rev = rev2008;
				}
				
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
				
				class ABGModelRobot
				{
					public:
						ABGModelRobot()
						{
							alphaPos = 1;
							betaPos = 1;
							gammaPos = 1;
							alphaAng = 1;
							betaAng = 1;
							gammaAng = 1;
						}

						float alphaPos;
						float betaPos;
						float gammaPos;
						float alphaAng;
						float betaAng;
						float gammaAng;

						void proc(QDomElement element);
				};
				ABGModelRobot abgModelRobot;

				class KalmanModelRobot
				{
					public:
						KalmanModelRobot()
						{
							covPosVel = 10;
							covVelAcc = 0.01;
							covPosAcc = 0.001;
							covPos = 10;
							covVel = 10;
							covAcc = 10;
							measurementNoise = 0.1;
						}

						float covPosVel;
						float covVelAcc;
						float covPosAcc;
						float covPos;
						float covVel;
						float covAcc;
						float measurementNoise;

						void proc(QDomElement element);
				};
				KalmanModelRobot kalmanModelRobot;

				class RBPFModelBallRolling
				{
				public:
					RBPFModelBallRolling()
					{
						processNoiseSqrdPos = 0.2;
						processNoiseSqrdVel = 1.0;
						processNoiseSqrdAcc = 1000.0;
						measurementNoiseSqrd = 0.01;
					}

					float processNoiseSqrdPos;
					float processNoiseSqrdVel;
					float processNoiseSqrdAcc;
					float measurementNoiseSqrd;

					void proc(QDomElement element);
				};
				RBPFModelBallRolling rbpfModelBallRolling;

				class RBPFModelBallKicked
				{
				public:
					RBPFModelBallKicked()
					{
						processNoiseSqrdPos = 1.0;
						processNoiseSqrdVel = 1.0;
						processNoiseSqrdAcc = 1000.0;
						measurementNoiseSqrd = 0.01;
					}

					float processNoiseSqrdPos;
					float processNoiseSqrdVel;
					float processNoiseSqrdAcc;
					float measurementNoiseSqrd;

					void proc(QDomElement element);
				};
				RBPFModelBallKicked rbpfModelBallKicked;
				
				void proc(QDomElement element);
		};

	public:
		typedef boost::shared_ptr<ConfigFile::Robot> shared_robot;
		typedef boost::shared_ptr<ConfigFile::WorldModel> shared_worldmodel;

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
		
		shared_worldmodel worldModel;
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

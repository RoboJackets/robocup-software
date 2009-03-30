#ifndef CONFIGFILE_HPP_
#define CONFIGFILE_HPP_

#include <QString>
#include <QDomDocument>
#include <QVector>
#include <QStringList>

#include <stdexcept>

#include <Geometry/Point2d.hpp>


class ConfigFile
{
    //types
    public:
        class linearControllerInfo
        {
        public:
            linearControllerInfo()
            {
                Kp = 0;
                Kd = 0;
            }
            
            float Kp, Kd;
            Geometry::Point2d deadband;
        };

        class RobotCfg
        {
        public:
            RobotCfg()
            {
                maxAccel = 0;
                maxWheelVel = 0;
                maxRobotVel = 0;
            }
            
            float maxAccel, maxWheelVel, maxRobotVel;

            linearControllerInfo posCntrlr, angleCntrlr;

            QVector<Geometry::Point2d> axels;
        };

        class robotKF
        {
        public:
            robotKF()
            {
                cmd_scale = 0;
                cmd_scale_angle = 0;
                bufsize = 0;
            }
            
            float cmd_scale;
            float cmd_scale_angle;
            int bufsize;
        };

        class RobotFilterCfg
        {
        public:
            RobotFilterCfg()
            {
                kf_self_enable = false;
                kf_opp_enable = false;
            }
            
            bool kf_self_enable;
            bool kf_opp_enable;
            robotKF self, opp;
        };

    public:
        ConfigFile(QString filename);
        ~ConfigFile();

        void load() throw(std::runtime_error);
        void save(QString filename = QString("")) throw (std::runtime_error);
        void setElement(QString tagString,int value);
        void setElement(QString tagString,double value);
        RobotCfg robotConfig();
        RobotFilterCfg robotFilterConfig();
        const QVector<Geometry::Point2d> axels() const { return _axels; }

    protected:
        /** returns the value of the attribute */
        static float valueFloat(QDomAttr attr);
        static int valueUInt(QDomAttr attr);
        static bool valueBool(QDomAttr attr);

        /** process a linear controller tag */
        void procLinearController(QDomElement element);
        /** process the axels */
        void procAxels(QDomElement element);
        /** process robot physical data **/
        void procRobotData(QDomElement element);

        /** process a world model tag */
        void procWorldModel(QDomElement element);
        /** process a RobotKF tag */
        void procRobotKF(QDomElement element);


    private:
        QString _filename;
        QDomDocument _doc;

        /** pid info for position and angle */
        linearControllerInfo _pos, _angle;

        float _maxAccel, _maxWheelVel, _maxRobotVel;
        /** axel positions */
        QVector<Geometry::Point2d> _axels;

        /** World Model general configuration */
        RobotFilterCfg _wm;
};

#endif /* CONFIGFILE_HPP_ */

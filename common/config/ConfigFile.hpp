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
        typedef struct
        {
            float Kp, Kd;
            Geometry::Point2d deadband;
        } linearControllerInfo;

        typedef struct
        {
            float maxAccel, maxWheelVel, maxRobotVel;

            linearControllerInfo posCntrlr, angleCntrlr;

            QVector<Geometry::Point2d> axels;
        } RobotCfg;

        typedef struct
        {
	    bool kf_self_enable;
	    bool kf_opp_enable;
        } RobotFilterCfg;

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

        /** process a linear controller tag */
        void procLinearController(QDomElement element);
        /** process the axels */
        void procAxels(QDomElement element);
        /** process robot physical data **/
        void procRobotData(QDomElement element);

    private:
        QString _filename;
        QDomDocument _doc;

        /** pid info for position and angle */
        linearControllerInfo _pos, _angle;

        float _maxAccel, _maxWheelVel, _maxRobotVel;
        /** axel positions */
        QVector<Geometry::Point2d> _axels;
};

#endif /* CONFIGFILE_HPP_ */

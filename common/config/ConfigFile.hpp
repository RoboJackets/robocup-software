#ifndef CONFIGFILE_HPP_
#define CONFIGFILE_HPP_

#include <QString>
#include <QDomDocument>
#include <QVector>

#include <stdexcept>

#include <Geometry/Point2d.hpp>


class ConfigFile
{
    //types
    public:
        typedef struct
        {
            float Kp, Kv;
            Geometry::Point2d deadband;
        } linearControllerInfo;

        typedef struct
        {
            unsigned int id;
            float maxAccel, maxWheelVel;

            linearControllerInfo posCntrlr, angleCntrlr;

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

        float _maxAccel, _maxWheelVel;
        /** axel positions */
        QVector<Geometry::Point2d> _axels;
};

#endif /* CONFIGFILE_HPP_ */

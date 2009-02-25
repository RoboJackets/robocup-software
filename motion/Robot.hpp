#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <Team.h>
#include <Geometry/TransformMatrix.hpp>

#include "ConfigFile.hpp"
#include "Pid.hpp"
#include "framework/Module.hpp"

class Robot
{
    private:
        /** robot velocity control information */
        typedef struct VelocityCmd
        {
        VelocityCmd() : w(0), maxWheelSpeed(127) {}

        /** velocity along x,y in team space */
        Geometry::Point2d vel;

        /** rotational velocity */
        float w;

        uint8_t maxWheelSpeed;
        } VelocityCmd;

    public:
        Robot(ConfigFile::RobotCfg cfg);
        ~Robot();

                void setSystemState(SystemState* state);

        /** Process the command for a robot and prepare output */
        void proc();

        /** clear PID windup */
        void clearPid();

                /*
        PathPlanner* pathPlanner() const { return _pathPlanner; }

        template <typename T>
        void newPathPlanner()
        {
            _pathPlanner = new T(_vision);
        }
                */
    private:
        /** given a created robot velocity, generate motor speeds */
        void genMotor(VelocityCmd velCmd);

        /** robot identification */
        const unsigned int _id;

        /** X pid **/
        Pid* _xPID;

        /** Y PID **/
        Pid* _yPID;

        /** angle pid **/
        Pid* _anglePID;

        float* _motors;


        /** team we are running as */
        Team _team;

        /** SystemState **/
        SystemState* _state;

                /*
        PathPlanner* _pathPlanner;
                */

        /** robot axels */
        QVector<Geometry::Point2d> _axels;

        /** transform matrix **/
        Geometry::TransformMatrix* rotationMatrix;
};

#endif /* ROBOT_HPP_ */

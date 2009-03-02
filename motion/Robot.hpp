#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <Team.h>
#include <Geometry/TransformMatrix.hpp>

#include <config/ConfigFile.hpp>
#include "Pid.hpp"
#include "framework/Module.hpp"
#include "LinearController.hpp"

class Robot
{
    private:
        /** robot velocity control information */
        typedef struct VelocityCmd
        {
            VelocityCmd() : w(0), maxWheelSpeed(127) {}

            /** velocity along x,y in the robot frame */
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

        /** motor values **/
        float* _motors;

        //These need to go directly to the linear controller class
        float _Kp, _Kv;

        /** Mechanical data from configfile **/
        float _maxAccel, _maxWheelVel;

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

        /** The wheel velocities in the last frame **/
        int8_t _lastWheelVel[4];

        /** Pos Controller **/
        LinearController::LinearController* _posController;

        /** DeadBand - needs to go in config file **/
        Geometry::Point2d _deadband;

        /** The max velocity the robot can have about the y or x is actually dependent on angle of motion**/
        float maxRobotVelocity;
};


#endif /* ROBOT_HPP_ */

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <Team.h>
#include <Geometry/TransformMatrix.hpp>

#include <config/ConfigFile.hpp>
#include "Pid.hpp"
#include "framework/Module.hpp"
#include "LinearController.hpp"
#include "PathPlanner.hpp"

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
        Robot(ConfigFile::RobotCfg cfg, unsigned int id);
        ~Robot();

        void setSystemState(SystemState* state)
        { _state = state; }


        /** Process the command for a robot and prepare output */
        void proc();

        /** Given a position generate a robot velocity **/
        void genVelocity();

        /** given a created robot velocity, generate motor speeds */
        void genMotor(VelocityCmd velCmd);

        /** clear PID windup */
        void clearPid();

        void setKp(double value);
        void setKd(double value);
                /*
        PathPlanner* pathPlanner() const { return _pathPlanner; }

        template <typename T>
        void newPathPlanner()
        {
            _pathPlanner = new T(_vision);
        }
                */
    private:

        /** robot identification */
        const unsigned int _id;

        /** motor values **/
        float* _motors;

        /** Mechanical data from configfile **/
        float _maxAccel, _maxWheelVel;

        /** team we are running as */
        Team _team;

        /** SystemState **/
        SystemState* _state;

        PathPlanner* _pathPlanner;

        /** robot axels */
        QVector<Geometry::Point2d> _axels;

        /** rotation matrix to go from team space to robot space**/
        Geometry::TransformMatrix* rotationMatrix;

        /** The wheel velocities in the last frame **/
        int8_t _lastWheelVel[4];

        /** Pos Controller **/
        LinearController::LinearController* _posController;

        /** DeadBand - needs to go in config file **/
        Geometry::Point2d _deadband;

        /** Path from pathplanner **/
        PathPlanner::Path _path;

        Geometry::Point2d _currPos;
        Geometry::Point2d _goalPos;
    };


#endif /* ROBOT_HPP_ */

#include "Trajectory.hpp"
#include <Constants.hpp>
#include <Team.h>
#include <math.h>

Trajectory::Trajectory(float maxA, float maxV)
{
    _maxA = maxA;
    _maxV = maxV;
    _tf = _t1 = _t12 = _t2 =_dt1 =_ax = _ay = _vx = _vy =  0;
}

Trajectory::~Trajectory()
{

}

Trajectory::TrajectoryCmd Trajectory::run(Geometry::Point2d currPos, float t)
{
    Geometry::Point2d cmdPos;
    Geometry::Point2d cmdVel;
    TrajectoryCmd trajCmd;
    _currPos = currPos;


    if(t < _t1)
    {
        cmdPos.x = _startPos.x + 0.5*_ax*(t*t);
        cmdVel.x = _ax*t;

        cmdPos.y = _startPos.y + 0.5*_ay*(t*t);
        cmdVel.y = _ay*t;

//         printf("Acceleration\n");
        printf("cmd %f %f\n",cmdPos.x,cmdPos.y);
//         printf("time %f\n",t);
//         printf("curr %f\n",currPos.x);
    }
    else if(t > _t1 && t < _t2)
    {
        cmdPos.x = _startPos.x + 0.5*_ax*(_t1*_t1) + _vx*t;
        cmdVel.x = _vx;

        cmdPos.y = _startPos.y + 0.5*_ay*(_t1*_t1) + _vy*t;
        cmdVel.y = _vy;

//         printf("Constant Velocity\n");
        printf("cmd %f\n",cmdPos.x);
//         printf("curr %f\n",currPos.x);
    }
    else if(t > _t2 && t < _tf)
    {
        t = t - _tf;
        cmdPos.x = _goalPos.x - 0.5*_ax*(t*t);
        cmdVel.x = _ax -_ax*t;

        cmdPos.y = _goalPos.y - 0.5*_ay*(t*t);
        cmdVel.y = _ay -_ay*t;

//         printf("Decceleration\n");
        printf("cmd %f %f\n",cmdPos.x,cmdPos.y);
//         printf("curr %f\n",currPos.x);
//         printf("time %f\n",t);
//         printf("TotalTime %f\n",_t1+_t12+_t2);
    }
    else
    {
        cmdPos = currPos;
        cmdVel.x = 0;
    }

    trajCmd.pos = cmdPos;
    trajCmd.v_ff = cmdVel;

    return trajCmd;
}

bool Trajectory::areWeThereYet()
{
    bool weThere = false;
    Geometry::Point2d error = _goalPos - _currPos;
    Geometry::Point2d deadband(0.05,0.05);
    printf("%f\n",error.x);
    if(error.x < deadband.x)
    {
        weThere = true;
    }
    return weThere;
}

void Trajectory::setTrajectory(Geometry::Point2d goalPoint,Geometry::Point2d startPos, Geometry::Point2d currVel)
{
    float alpha = M_PI/4, alpha_min = 0, alpha_max = M_PI/2;
    //Cos(alpha) Sin(alpha)
    float c_a, s_a;
    float lastAlpha = 0;
    Geometry::Point2d trajLength;
    TrajectoryParams xParams, yParams;

    //Reset variables
    _goalPos = goalPoint;
    _startPos = startPos;
    _vel = currVel;

    trajLength.x = fabs(_startPos.x - _goalPos.x);
    trajLength.y = fabs(_startPos.y - _goalPos.y);

    if(trajLength.x < 0.01 )
    {
        //Don't attempt the bisection algorithm its going to be a 1D ride for now
        c_a = cos(alpha);
        xParams = calcTime(c_a*_maxA, c_a*_maxV, trajLength.x);
        _t1 = xParams.t1;
        _t12 = xParams.t12;
        _t2 = xParams.t2;
        _tf = xParams.tf;
        _dt1 = xParams.dt1;
        _ax = c_a*_maxA;
        _ay = 0;
        _vx = c_a*_maxV;
        _vy = 0;
        return;
    }

    if(trajLength.y < 0.01 )
    {
        //Don't attempt the bisection algorithm its going to be a 1D ride for now
        s_a = cos(alpha);
        yParams = calcTime(s_a*_maxA, s_a*_maxV, trajLength.y);
        _t1 = yParams.t1;
        _t12 = yParams.t12;
        _t2 = yParams.t2;
        _tf = yParams.tf;
        _dt1 = yParams.dt1;
        _ax = 0;
        _ay = s_a*_maxA;
        _vx = 0;
        _vy = s_a*_maxV;
        return;
    }

    //Bisection Algorithm
    while(1)
    {
        c_a = cos(alpha);
        s_a = sin(alpha);
        xParams = calcTime(c_a*_maxA, c_a*_maxV, trajLength.x);
        yParams = calcTime(s_a*_maxA, s_a*_maxV, trajLength.y);
//         printf("alpha %f\n",alpha);
//         printf("error %f\n", fabs(xParams.tf - yParams.tf));
        printf("Tx %f Ty %f\n",xParams.tf ,yParams.tf);
        printf("alpha %f\n",alpha);
        if(fabs(xParams.tf - yParams.tf) <= 0.02 || fabs(lastAlpha - alpha) < 0.0001 || alpha == 0)
        {
            _t1 = xParams.t1;
            _t12 = xParams.t12;
            _t2 = xParams.t2;
            _tf = xParams.tf;
            _dt1 = xParams.dt1;
            _ax = c_a*_maxA;
            _ay = s_a*_maxA;
            _vx = c_a*_maxV;
            _vy = s_a*_maxV;
            break;
        }
        else if(xParams.tf > yParams.tf)
        {
            lastAlpha = alpha;
            alpha = 0.5*(alpha - alpha_min);
            alpha_max = alpha;
        }
        else if(xParams.tf < yParams.tf)
        {
            lastAlpha = alpha;
            alpha = 0.5*(alpha_max - alpha);
            alpha_min = alpha;

        }
    }
}

Trajectory::TrajectoryParams Trajectory::calcTime(float maxA, float maxV, float length)
{
    TrajectoryParams trajParam;
    trajParam.t1 = maxV/maxA;
    trajParam.dt1 = 0.5*maxA*(trajParam.t1*trajParam.t1);
    trajParam.t12 = (length - (2 * trajParam.dt1))/ maxV;
    trajParam.tf = 2*trajParam.t1 + trajParam.t12;
    trajParam.t2 = trajParam.t12 + trajParam.t1;

    if(trajParam.dt1 > length)
    {
        //Robot won't reach max speed in this direction over the length of the trajectory
        //Switch to triangular velocity profile
        trajParam.t1 = sqrt(length/maxA);
        trajParam.dt1 = 0.5*maxA*(trajParam.t1*trajParam.t1);
        trajParam.t12 = 0;
        trajParam.t2 = trajParam.t1;
        trajParam.tf = 2*trajParam.t1;
    }

    return trajParam;
}
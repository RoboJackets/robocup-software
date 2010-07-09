#pragma once

#include <vector>
#include <string>

#include <framework/Vision.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Circle.hpp>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <Team.h>
#include <GameState.hpp>
#include <framework/MotionCmd.hpp>
#include <framework/Robot.hpp>

    class SystemState
    {
        public:
            class Robot
             : public Framework::Robot
            {
                public:
                    uint8_t shell;
                    enum Rev
                    {
                        rev2008 = 0,
                        rev2010 = 1
                    };
                    
                    Rev rev;
                    Geometry2d::Point pos;
                    Geometry2d::Point vel;
                    float angle;
                    float angleVel;
                    MotionCmd cmd;
                    bool valid;
                    bool haveBall;
                    Geometry2d::Point cmd_vel;
                    float cmd_w;
                    RadioTx::Robot radioTx;
                    Packet::RadioRx radioRx;
                    class Pose
                    {
                        public:
                            Geometry2d::Point pos;
                            float angle;
                            
                            Pose()
                            {
                                angle = 0;
                            }
                    };
                    
                    std::vector<Pose> poseHistory;
                    
                    Robot()
                    {
                        shell = 0;
                        rev = rev2008;
                        angle = 0;
                        angleVel = 0;
                        valid = false;
                        haveBall = false;
                        cmd_w = 0;
                    }
            };
            
            class Ball
            {
                public:
                    Geometry2d::Point pos;
                    Geometry2d::Point vel;
                    Geometry2d::Point accel;
                    bool valid;
                    
                    Ball()
                    {
                        valid = false;
                    }
            };
            
            enum Possession
            {
                OFFENSE = 0,
                DEFENSE = 1,
                FREEBALL = 2
            };
            
            enum BallFieldPos
            {
                HOMEFIELD = 0,
                MIDFIELD = 1,
                OPPFIELD = 2
            };
            
            class GameStateID
            {
                public:
                    Possession posession;
                    BallFieldPos field_pos;
                    
                    GameStateID()
                    {
                        posession = OFFENSE;
                        field_pos = HOMEFIELD;
                    }
            };
            
            GameStateID stateID;
            uint64_t timestamp;
            Team team;
            bool autonomous;
            int8_t manualID;
            std::string playName;
            std::vector<Packet::Vision> rawVision;
            GameState gameState;
            Robot self[5];
            Robot opp[5];
            Ball ball;
            std::vector<Geometry2d::Point> pathTest;
            class DebugPolygon
             : public Geometry2d::Polygon
            {
                public:
                    uint8_t color[3];
                    
                    DebugPolygon()
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            color[i] = 0;
                        }
                    }
            };
            
            class DebugLine
             : public Geometry2d::Segment
            {
                public:
                    uint8_t color[3];
                    
                    DebugLine()
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            color[i] = 0;
                        }
                    }
            };
            
            class DebugCircle
             : public Geometry2d::Circle
            {
                public:
                    uint8_t color[3];
                    
                    DebugCircle()
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            color[i] = 0;
                        }
                    }
            };
            
            class DebugText
            {
                public:
                    std::string text;
                    Geometry2d::Point pos;
                    uint8_t color[3];
                    
                    DebugText()
                    {
                        for (int i = 0; i < 3; ++i)
                        {
                            color[i] = 0;
                        }
                    }
            };
            
            std::vector<DebugPolygon> debugPolygons;
            std::vector<DebugLine> debugLines;
            std::vector<DebugCircle> debugCircles;
            std::vector<DebugText> debugText;
            
            LogFrame()
            {
                timestamp = 0;
                team = UnknownTeam;
                autonomous = 0;
            }
    };


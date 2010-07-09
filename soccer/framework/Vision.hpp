#pragma once

#include <vector>
#include <string>
#include <stdint.h>

#include <Geometry2d/Point.hpp>

namespace Packet
{
    class Vision
    {
        public:
            class Robot
            {
                public:
                    uint8_t shell;
                    Geometry2d::Point pos;
                    float angle;
                    
                    Robot()
                    {
                        shell = 0;
                        angle = 0;
                    }
            };
            
            class Ball
            {
                public:
                    Geometry2d::Point pos;
                    
                    Ball()
                    {
                    }
            };
            
            uint64_t timestamp;
            uint8_t camera;
            bool sync;
            std::vector<Ball> balls;
            std::vector<Robot> blue;
            std::vector<Robot> yellow;
            
            Vision()
            {
                timestamp = 0;
                camera = 0;
                sync = false;
            }
    };
    
}

